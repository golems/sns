/*
 * Copyright (c) 2015-2017 Rice University
 *
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

#include <stdint.h>
#include <amino.h>
#include <ach.h>
#include <ach/experimental.h>
#include <getopt.h>
#include <stdbool.h>

#include <amino/rx/rxtype.h>
#include <amino/rx/scenegraph.h>
#include <amino/rx/scene_plugin.h>

#include "sns.h"
#include "sns/event.h"

#include <amino/rx/scene_collision.h>
#include <cblas.h>
#include <stdio.h>



struct cx {
    struct in_cx *in;

    struct ach_channel ref_out;

    struct aa_rx_sg *scenegraph;

    struct sns_evhandler handlers[2];
    struct ach_channel channel[2];

    struct timespec period;
    struct timespec t;

    size_t n_q;

    double *q_act;
    double *dq_act;

    double *q_ref;
    double *dq_ref;
    int have_q_ref;
    int have_dq_ref;
};


enum ach_status handle_ref_in( void *cx, void *msg, size_t msg_size );
enum ach_status handle_state( void *cx, void *msg, size_t msg_size );

void send_ref( struct cx *cx );

int test_for_collisions( struct cx *cx );

int main(int argc, char **argv)
{
    struct cx cx = {0};
    /* parse options */
    double opt_frequency = 100;
    const char *opt_chan_state = NULL;
    const char *opt_chan_ref = NULL;
    const char *opt_chan_ref_in = NULL;
    const char *opt_scene_plugin = NULL;
    const char *opt_scene_name = NULL;
    {
        int c = 0;
        while( (c = getopt( argc, argv, "u:y:j:s:n:h?" SNS_OPTSTRING)) != -1 ) {
            switch(c) {
                SNS_OPTCASES_VERSION("sns-watchdog",
                                     "Copyright (c) 2015-2017, Rice University\n",
                                     "Neil T. Dantam")
            case 's':
                opt_scene_plugin = optarg;
                break;
            case 'n':
                opt_scene_name = optarg;
                break;
            case 'u':
                opt_chan_ref = optarg;
                break;
            case 'y':
                opt_chan_state = optarg;
                break;
            case 'j':
                opt_chan_ref_in = optarg;
                break;
            case '?':   /* help     */
            case 'h':
                puts( "Usage: sns-watchdog -j <ref_in-channel> -u <ref_out-channel> -y <state-channel>\n"
                      "Teleop a robot.\n"
                      "\n"
                      "Options:\n"
                      "  -y <channel>,             state channel, input\n"
                      "  -j <channel>,             reference channel, input\n"
                      "  -u <channel>,             reference channel, output\n"
                      "  -s <plugin>,              scene plugin\n"
                      "  -n <name>,                scene name\n"
                      "  -V,                       Print program version\n"
                      "  -?,                       display this help and exit\n"
                      "\n"
                      "Examples:\n"
                      "  sns-teleopd -j ref_in -y state -u ref\n"
                      "\n"
                      "Report bugs to " PACKAGE_BUGREPORT );
                      exit(EXIT_SUCCESS);
                default:
                      SNS_DIE("Unknown Option: `%c'\n", c);
                      break;
            }
        }
    }
    sns_init();
    SNS_REQUIRE(opt_chan_state, "Need state channel");
    SNS_REQUIRE(opt_chan_ref, "Need ref_out channel");
    SNS_REQUIRE(opt_chan_ref_in, "Need ref_in channel");
    SNS_REQUIRE(opt_scene_plugin, "Need scene plugin");
    SNS_REQUIRE(opt_scene_name, "Need scene name");

    /* Load Scene Plugin */
    cx.scenegraph = aa_rx_dl_sg(opt_scene_plugin, opt_scene_name, NULL);
    SNS_REQUIRE( NULL != cx.scenegraph, "Could not load scene plugin");
    aa_rx_sg_init(cx.scenegraph);
    cx.n_q = aa_rx_sg_config_count(cx.scenegraph);
    cx.q_act = AA_NEW_AR(double,cx.n_q);
    cx.dq_act = AA_NEW_AR(double,cx.n_q);
    cx.q_ref = AA_NEW_AR(double,cx.n_q);
    cx.dq_ref = AA_NEW_AR(double,cx.n_q);

    /* Setup channels */
    sns_chan_open( &cx.channel[0], opt_chan_state, NULL );
    cx.handlers[0].channel = &cx.channel[0];
    cx.handlers[0].context = &cx;
    cx.handlers[0].handler = handle_state;
    cx.handlers[0].ach_options = ACH_O_LAST;

    sns_chan_open( &cx.channel[1], opt_chan_ref_in, NULL );
    cx.handlers[1].channel = &cx.channel[1];
    cx.handlers[1].context = &cx;
    cx.handlers[1].handler = handle_ref_in;
    cx.handlers[1].ach_options = 0;

    sns_chan_open( &cx.ref_out, opt_chan_ref, NULL );


    printf("about to start event loop\n");
    
    /* Start Event Loop */
    cx.period = aa_tm_sec2timespec( 1 / opt_frequency );
    sns_start();
    enum ach_status r =
        sns_evhandle( cx.handlers, 2,
                      &cx.period, NULL, NULL,
                      sns_sig_term_default,
                      ACH_EV_O_PERIODIC_TIMEOUT );
    SNS_REQUIRE( sns_cx.shutdown || (ACH_OK == r),
                 "Could not handle events: %s, %s\n",
                 ach_result_to_string(r),
                 strerror(errno) );
    /* Halt */
    AA_MEM_ZERO(cx.q_ref, cx.n_q);
    send_ref(&cx);

    sns_end();

    return 0;
}

enum ach_status handle_ref_in( void *cx_, void *msg_, size_t msg_size )
{

    printf("ref in\n");
    struct cx *cx = (struct cx*)cx_;
    struct sns_msg_motor_ref *msg = (struct sns_msg_motor_ref *)msg_;

    if( sns_msg_motor_ref_check_size(msg,msg_size) ) {
        /* Invalid Message */
        SNS_LOG(LOG_ERR, "Mistmatched message size on channel\n");
    } else {
        /* Process Message */
        SNS_LOG(LOG_DEBUG, "Got a message for ref_in\n")

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

        if(test_for_collisions(cx)) {
            send_ref(cx);
        }
    }
    return ACH_OK;
}

enum ach_status handle_state( void *cx_, void *msg_, size_t msg_size )
{
    printf("state in\n");
    struct cx *cx = (struct cx*)cx_;
    struct sns_msg_motor_state *msg = (struct sns_msg_motor_state *)msg_;
    
    if( sns_msg_motor_state_check_size(msg,msg_size) ) {
        /* Invalid Message */
        SNS_LOG(LOG_ERR, "Mistmatched message size on channel\n");
    } else {
        /* Process Message */
        SNS_LOG(LOG_DEBUG, "Got a message on state channel\n")

        for( size_t i = 0; i < cx->n_q && i < msg->header.n; i++) {
            cx->q_act[i] = msg->X[i].pos;
	    cx->dq_act[i] = msg->X[i].vel;
        }
    }
    return ACH_OK;
}

int test_for_collisions( struct cx *cx ) {
    printf("testing collisions\n");
    struct timespec now;
    clock_gettime(ACH_DEFAULT_CLOCK, &now);
    //TODO: Does this really make sense?
    double dt = aa_tm_timespec2sec( aa_tm_sub(now, cx->t) );
    //For now we use this:
    dt = 0.01;
    cx->t = now;
    int n_q = (int)cx->n_q;

    double *q_act = cx->q_act;

    /*
    // Set Refs
    if( cx->have_q_ref ) {
        // Set ref pos
        cblas_dcopy( n_q, cx->q_ref, 1, cx->q_act, 1 );
        AA_MEM_ZERO(cx->dq_act, cx->n_q);
    } else if( cx->have_dq_ref ) {
        // Set ref vel
        cblas_dcopy( n_q, cx->dq_ref, 1, cx->dq_act, 1 );
    }
    cx->have_q_ref = 0;
    cx->have_dq_ref = 0;
    */
    // Integrate (euler step)
    
    double *q_act_copy;
    q_act_copy = (double*) aa_mem_region_local_alloc(sizeof(cx->q_act[0]) * (size_t)n_q);
    memcpy(q_act_copy, cx->q_act, sizeof(cx->q_act[0]) * (size_t)n_q);
    //TODO: what should the time step be?
    cblas_daxpy(n_q, 2*dt, cx->dq_act, 1, cx->q_act, 1);
    
    struct aa_rx_sg *scenegraph = cx->scenegraph;//sns_scene_load();
    aa_rx_sg_cl_init(scenegraph);
    aa_rx_sg_init(scenegraph);
    struct aa_rx_cl *cl = aa_rx_cl_create(scenegraph);

    //aa_rx_sg_cl_init(scenegraph);
    cx->have_q_ref = 0;
    cx->have_dq_ref = 0;
    //check for collisions
    //
    size_t n_tf = aa_rx_sg_frame_count(scenegraph);

    
    if((size_t)n_q != aa_rx_sg_config_count(scenegraph)) {
        printf("n_q not set correctly.");
    }
        
    printf("n_q = %d; n_tf = %lu, dt = %f\n", n_q, n_tf, dt);

    double *TF = (double*) aa_mem_region_local_alloc(14*n_tf*sizeof(double));
    double *TF_rel = TF, *TF_abs = TF+7;
    aa_rx_sg_tf(scenegraph, (size_t)n_q, q_act, n_tf, TF_rel, 14, TF_abs, 14);

    /*
    for(size_t i = 0; i < 7*n_tf; i++) {
      printf("TF_rel[%lu] = %f; TF_abs[%lu] = %f\n", i, TF_rel[i], i, TF_abs[i]);
    }
    */
    
    if( aa_rx_cl_check(cl, n_tf, TF_abs, 14, NULL)) {
        memcpy(cx->q_act, q_act_copy, sizeof(cx->q_act[0]) * (size_t)n_q);
        aa_mem_region_local_pop(TF);
        aa_mem_region_local_pop(q_act_copy);
        printf("collision found\n");
        return 1;//dt = 0.0; //send message to halt
    }
    
    memcpy(cx->q_act, q_act_copy, sizeof(cx->q_act[0]) * (size_t)n_q);
    aa_mem_region_local_pop(TF);
    aa_mem_region_local_pop(q_act_copy);
    printf("collision not found\n");
    return 0;
}

void send_ref( struct cx *cx )
{
    printf("halting robot\n");
    struct sns_msg_motor_ref *msg = sns_msg_motor_ref_local_alloc((uint32_t)cx->n_q);
    struct timespec now;
    clock_gettime( ACH_DEFAULT_CLOCK, &now );
    sns_msg_set_time( &msg->header, &now, 1e9 ); /* 1 second duration */

    msg->mode = SNS_MOTOR_MODE_HALT;

    for(size_t i = 0; i < cx->n_q; i++) {
        cx->dq_ref[i] = 0.0;
    }
    
    AA_MEM_CPY(msg->u, cx->dq_ref, cx->n_q);

    enum ach_status r = sns_msg_motor_ref_put(&cx->ref_out, msg);
    if( ACH_OK != r )  {
        SNS_LOG( LOG_ERR, "Failed to put message: %s\n", ach_result_to_string(r) );
    }

    aa_mem_region_local_pop(msg);

}
