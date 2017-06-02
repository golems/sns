/*
 * Copyright (c) 2017, Rice University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products
 *       derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "config.h"

#include <poll.h>


#include <getopt.h>

#include <cblas.h>

#include <amino.h>
#include <amino/rx/rxtype.h>
#include <amino/rx/scenegraph.h>
#include <amino/rx/scene_plugin.h>
#include <amino/ct/state.h>

#include <amino/rx/scene_gl.h>
#include <amino/rx/scene_win.h>

#include "sns.h"
#include <sns/event.h>
#include <ach/experimental.h>
#include <sns/motor.h>

struct cx;


struct in_cx {
    struct ach_channel channel;
    const char *name;
    struct cx *cx;
};

struct cx {
    struct in_cx *in;
    struct ach_channel state_out;
    size_t n_ref;

    struct aa_rx_sg *scenegraph;


    double *q_ref;
    double *dq_ref;
    int have_q_ref;
    int have_dq_ref;
    struct timespec t;

    //double *q_act;
    //double *dq_act;

    struct aa_ct_state *state_act;

    size_t n_q;
    uint64_t seq;

    struct aa_rx_win * win;

    struct sns_evhandler *handlers;
    struct timespec period;

};

// Handle a message
enum ach_status handle_msg( void *cx, void *msg, size_t msg_size );

// Run io
void io(struct cx *cx);
// Pthreads start function for io
void* io_start(void *cx);

// Call periodically from io thread
enum ach_status io_periodic( void *cx );

void put_state( struct cx *cx );

// Perform a simulation step
enum ach_status simulate( struct cx *cx );

int main(int argc, char **argv)
{
    struct cx cx;
    AA_MEM_ZERO(&cx,1);

    struct aa_mem_rlist *names_list = aa_mem_rlist_alloc( aa_mem_region_local_get() );

    const char *opt_chan_state = NULL;
    const char *opt_scene_plugin = NULL;
    const char *opt_scene_name = NULL;
    const double opt_sim_frequecy = 100;

    /* Parse Options */
    {
        int c = 0;
        opterr = 0;
        while( (c = getopt( argc, argv, "s:o:i:n:h?" SNS_OPTSTRING)) != -1 ) {
            switch(c) {
                SNS_OPTCASES_VERSION("sns-ksim",
                                     "Copyright (c) 2017, Rice University\n",
                                     "Neil T. Dantam")
            case 's':
                opt_scene_plugin = optarg;
                break;
            case 'n':
                opt_scene_name = optarg;
                break;
            case 'o':
                opt_chan_state = optarg;
                break;
            case 'i':
                aa_mem_rlist_push_ptr( names_list, optarg );
                cx.n_ref++;
                break;
            case '?':   /* help     */
            case 'h':
                puts( "Usage: sns-ksim -i INPUT_CHANNEL -o OUTPUT_CHANNEL -s SCENE_PLUGIN\n"
                      "Kinematically simulate a robot.\n"
                      "\n"
                      "Options:\n"
                      "  -o,                       state output channel\n"
                      "  -i,                       reference input channel\n"
                      "  -s,                       scenegraph plugin\n"
                      "  -n,                       scenegraph name\n"
                      "  -V,                       Print program version\n"
                      "  -?,                       display this help and exit\n"
                      "\n"
                      "Examples:\n"
                      "  sns-ksim -o state -i ref -s libmyrobot.so -n myrobot\n"
                      "\n"
                      "Report bugs to <ntd@rice.edu>"
                    );
                exit(EXIT_SUCCESS);
            default:
                SNS_DIE("Unknown Option: `%c'\n", c);
                break;
            }
        }
    }
    sns_init();

    /* state channel */
    SNS_REQUIRE( opt_chan_state, "Need output channel");
    SNS_LOG(LOG_INFO, "State Channel: `%s'\n", opt_chan_state);
    sns_chan_open( &cx.state_out, opt_chan_state , NULL );

    /* Scene Plugin */
    SNS_REQUIRE( NULL != opt_scene_plugin, "Need a scene plugin");
    SNS_REQUIRE( NULL != opt_scene_name, "Need a scene name");
    cx.scenegraph = aa_rx_dl_sg(opt_scene_plugin, opt_scene_name, NULL);
    SNS_REQUIRE( NULL != cx.scenegraph, "Could not load scene plugin");
    aa_rx_sg_init(cx.scenegraph);
    cx.n_q = aa_rx_sg_config_count(cx.scenegraph);

    cx.state_act = AA_NEW0(struct aa_ct_state);
    cx.state_act->q = AA_NEW_AR(double,cx.n_q);
    cx.state_act->dq = AA_NEW_AR(double,cx.n_q);
    cx.state_act->n_q = cx.n_q;

    cx.q_ref = AA_NEW_AR(double,cx.n_q);
    cx.dq_ref = AA_NEW_AR(double,cx.n_q);

    clock_gettime(ACH_DEFAULT_CLOCK, &cx.t);

    cx.handlers = AA_NEW_AR( struct sns_evhandler, cx.n_ref);
    cx.in = AA_NEW_AR(struct in_cx, cx.n_ref);

    // Initialize arrays
    for( size_t j = cx.n_ref; j; j--) {
        size_t i = j-1;

        const char *name = (const char*)aa_mem_rlist_pop(names_list);
        SNS_LOG(LOG_INFO, "Reference Channel[%lu]: `%s'\n", i, name);

        cx.in[i].name = name;
        cx.in[i].cx = &cx;

        // open channel
        sns_chan_open( &cx.in[i].channel, cx.in[i].name, NULL );

        // init handler
        cx.handlers[i].channel = &cx.in[i].channel;
        cx.handlers[i].context = cx.in+i;
        cx.handlers[i].handler = handle_msg;
        cx.handlers[i].ach_options = ACH_O_LAST;

    }

    SNS_LOG(LOG_INFO, "Simulation Frequency: %.3fkHz\n", opt_sim_frequecy/1e3);
    long period_ns = (long)(1e9 / opt_sim_frequecy);

    cx.period.tv_sec = (time_t)period_ns / (time_t)1e9;
    cx.period.tv_nsec = period_ns % (long)1e9;

    SNS_LOG( LOG_DEBUG, "Simulation Period: %lus + %ldns\n",
             cx.period.tv_sec, cx.period.tv_nsec );


    // Start threads
    pthread_t io_thread;
    if( pthread_create(&io_thread, NULL, io_start, &cx) ) {
        SNS_DIE("Could not create simulation thread: `%s'", strerror(errno));
    }

    // Start GUI in main thread
    cx.win = aa_rx_win_default_create ( "sns-ksim", 800, 600 );
    aa_rx_win_set_sg(cx.win, cx.scenegraph);
    sns_start();
    aa_rx_win_run();

    // Stop threads
    sns_cx.shutdown = 1;
    if( pthread_join(io_thread, NULL) ) {
        SNS_LOG(LOG_ERR, "Could not join simulation thread: `%s'", strerror(errno));
    }
    sns_end();



    return 0;
}


void* io_start(void *cx) {
    io((struct cx*)cx);
    return NULL;
}

void io(struct cx *cx) {
    // Run Loop
    enum ach_status r = sns_evhandle( cx->handlers, cx->n_ref,
                                      &cx->period, io_periodic, cx,
                                      sns_sig_term_default,
                                      ACH_EV_O_PERIODIC_TIMEOUT );
    SNS_REQUIRE( sns_cx.shutdown || (ACH_OK == r),
                 "Could asdf not handle events: %s, %s\n",
                 ach_result_to_string(r),
                 strerror(errno) );
    // stop window
    if( cx->win ) {
        aa_rx_win_stop(cx->win);
    }
}

enum ach_status handle_msg( void *cx_, void *msg_, size_t frame_size )
{
    struct sns_msg_motor_ref *msg = (struct sns_msg_motor_ref *)msg_;
    struct in_cx *cx_in = (struct in_cx*)cx_;
    struct cx *cx = cx_in->cx;

    if( frame_size < sizeof(struct sns_msg_header) ) {
        SNS_LOG(LOG_ERR, "Invalid message size on channel\n");
    } else if( sns_msg_motor_ref_check_size(msg,frame_size) ) {
        SNS_LOG(LOG_ERR, "Mistmatched message size on channel\n");
    } else if( msg->header.n != cx->n_q ) {
        SNS_LOG(LOG_ERR, "Mistmatched element count in reference message\n");
    } else {
        // Message looks OK
        SNS_LOG(LOG_DEBUG, "Got a message on channel %s\n", cx_in->name )
            switch(msg->mode) {
            case SNS_MOTOR_MODE_POS:
                for( size_t i = 0; i < cx->n_q; i ++ ) {
                    cx->q_ref[i] = msg->u[i];
                }
                cx->have_q_ref = 1;
                break;
            case SNS_MOTOR_MODE_VEL:
                for( size_t i = 0; i < cx->n_q; i ++ ) {
                    cx->dq_ref[i] = msg->u[i];
                }
                cx->have_dq_ref = 1;
                break;
            default:
                SNS_LOG(LOG_WARNING, "Unhandled motor mode: `%s'", sns_motor_mode_str(msg->mode));
            }
    }

    return ACH_OK;
}

enum ach_status io_periodic( void *cx_ )
{
    struct cx *cx = (struct cx*)cx_;
    // Run simulation
    simulate(cx);
    put_state(cx);


    // Update display
    if( cx->win ) aa_rx_win_set_config( cx->win, cx->n_q, cx->state_act->q );

    // check cancelation
    if( sns_cx.shutdown ) {
        return ACH_CANCELED;
    } else {
        return ACH_OK;
    }
}


enum ach_status simulate( struct cx *cx )
{
    struct timespec now;
    clock_gettime(ACH_DEFAULT_CLOCK, &now);
    double dt = aa_tm_timespec2sec( aa_tm_sub(now, cx->t) );
    cx->t = now;
    int n_q = (int)cx->n_q;

    // Set Refs
    if( cx->have_q_ref ) {
        // Set ref pos
        cblas_dcopy( n_q, cx->q_ref, 1, cx->state_act->q, 1 );
        AA_MEM_ZERO(cx->state_act->dq, cx->n_q);
    } else if( cx->have_dq_ref ) {
        // Set ref vel
        cblas_dcopy( n_q, cx->dq_ref, 1, cx->state_act->dq, 1 );
    }
    cx->have_q_ref = 0;
    cx->have_dq_ref = 0;

    // Integrate (euler step)
    cblas_daxpy(n_q, dt, cx->state_act->dq, 1, cx->state_act->q, 1 );

    return ACH_OK;
}

void put_state( struct cx *cx )
{
    sns_motor_map_state_out( cx->state_act, NULL,
                             &cx->t, (int64_t)1e9,
                             &cx->state_out );

}
