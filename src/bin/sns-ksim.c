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


#include <poll.h>
#include <sns.h>
#include <sns/event.h>
#include <ach/experimental.h>
#include <getopt.h>

#include <amino/rx/rxtype.h>
#include <amino/rx/scenegraph.h>
#include <amino/rx/scene_plugin.h>

#include <amino/rx/scene_gl.h>
#include <amino/rx/scene_win.h>

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

    double *q_act;
    double *dq_act;
    size_t n_q;

    struct aa_rx_win * win;

    struct ach_evhandler *handlers;
    struct timespec period;
};

void io(struct cx *cx);
void* io_start(void *cx);
enum ach_status simulate( void *cx_ );

enum ach_status handle( void *cx, struct ach_channel *channel );
enum ach_status periodic( void *cx );

int main(int argc, char **argv)
{
    sns_init();

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
        while( (c = getopt( argc, argv, "s:o:i:n:h?V" SNS_OPTSTRING)) != -1 ) {
            switch(c) {
                SNS_OPTCASES
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
                      "  sns-mplex -chan -o out-chan input-0 input-1 input-2\n"
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
    cx.q_act = AA_NEW_AR(double,cx.n_q);
    cx.dq_act = AA_NEW_AR(double,cx.n_q);
    cx.q_ref = AA_NEW_AR(double,cx.n_q);
    cx.dq_ref = AA_NEW_AR(double,cx.n_q);

    cx.handlers = AA_NEW_AR( struct ach_evhandler, cx.n_ref);
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
        cx.handlers[i].handler = handle;

    }

    SNS_LOG(LOG_INFO, "Simulation Frequeny: `%f'\n", opt_sim_frequecy);
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

    // Start GUI
    cx.win = aa_rx_win_default_create ( "sns-ksim", 800, 600 );

    aa_rx_win_set_sg(cx.win, cx.scenegraph);
    aa_rx_win_run();

    // Stop threads
    sns_cx.shutdown = 1;
    if( pthread_join(io_thread, NULL) ) {
        SNS_LOG(LOG_ERR, "Could not join simulation thread: `%s'", strerror(errno));
    }

    return 0;
}


void* io_start(void *cx) {
    io((struct cx*)cx);
    return NULL;
}

void io(struct cx *cx) {
    // Run Loop
    while( !sns_cx.shutdown) {
        errno = 0;
        printf("evhandle\n");
        enum ach_status r = ach_evhandle( cx->handlers, cx->n_ref,
                                          &cx->period, simulate, cx,
                                          ACH_EV_O_PERIODIC_TIMEOUT );
        if(sns_cx.shutdown) break;
        SNS_REQUIRE( ACH_OK == r,
                     "Could not handle events: %s, %s\n",
                     ach_result_to_string(r),
                     strerror(errno) );
    }
}


enum ach_status handle( void *cx_, struct ach_channel *channel )
{
    struct in_cx *cx_in = (struct in_cx*)cx_;
    struct cx *cx = cx_in->cx;

    //SNS_LOG(LOG_DEBUG, "Event on channel %s\n", cx_in->name );

    struct sns_msg_motor_ref *msg;
    size_t frame_size;
    enum ach_status r = sns_msg_motor_ref_local_get( channel, &msg,
                                                     &frame_size, NULL, ACH_O_LAST );
    if( ACH_STALE_FRAMES == r ) return r;
    SNS_REQUIRE( (ACH_OK == r || ACH_MISSED_FRAME == r),
                 "Could not get ach message on: %s\n",
                 ach_result_to_string(r) );

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
        default:
            SNS_LOG(LOG_WARNING, "Unhandled motor mode: `%s'", sns_motor_mode_str(msg->mode));
        }
    }


    aa_mem_region_local_pop( msg );

    return ACH_OK;
}

enum ach_status simulate( void *cx_ ) {
    struct cx *cx = (struct cx*)cx_;
    if( cx->have_q_ref ) {
        AA_MEM_CPY( cx->q_act, cx->q_ref, cx->n_q );
        if( cx->win ) aa_rx_win_set_config( cx->win, cx->n_q, cx->q_act );
        cx->have_q_ref = 0;
    }
    return ACH_OK;
}
