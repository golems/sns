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
#include <amino/ct/state.h>
#include <ach.h>
#include <ach/experimental.h>
#include <getopt.h>
#include <stdbool.h>

#include <amino/rx/rxtype.h>
#include <amino/rx/scenegraph.h>
#include <amino/rx/scene_plugin.h>

#include <sns.h>
#include <sns/event.h>
#include <sns/motor.h>

#include <amino/rx/scene_collision.h>
#include <cblas.h>
#include <stdio.h>

struct cx {
    struct sns_motor_channel *ref_out;
    struct sns_motor_ref_set *ref_set_out;

    struct sns_motor_channel *ref_in;
    struct sns_motor_ref_set *ref_set_in;

    struct sns_motor_channel *state_in;
    struct sns_motor_state_set *state_set;

    struct aa_rx_sg *scenegraph;

    struct sns_evhandler *handlers;

    struct timespec period;
    struct timespec t;

    size_t n_q;
};

enum ach_status handle_ref_in( void *cx, void *msg, size_t msg_size );
enum ach_status handle_state( void *cx, void *msg, size_t msg_size );
enum ach_status periodic(void * cx_);

void send_ref( struct cx *cx );

int test_for_collisions( struct cx *cx, struct timespec now );

int main(int argc, char **argv)
{
    struct cx cx = {0};
    /* parse options */
    double opt_frequency = 100;
    {
        int c = 0;
        while( (c = getopt( argc, argv, "u:y:j:s:n:h?" SNS_OPTSTRING)) != -1 ) {
            switch(c) {
                SNS_OPTCASES_VERSION("sns-watchdog",
                                     "Copyright (c) 2015-2017, Rice University\n",
                                     "Neil T. Dantam")
            case 'u':
                sns_motor_channel_push(optarg, &cx.ref_out);
                break;
            case 'y':
                sns_motor_channel_push(optarg, &cx.state_in);
                break;
            case 'j':
                sns_motor_channel_push(optarg, &cx.ref_in);
                break;
            case '?':   /* help     */
            case 'h':
                puts( "Usage: sns-watchdog -j <ref_in-channel> -u <ref_out-channel> -y <state-channel>\n"
                      "Watches for robot collisions and stops if there are any.\n"
                      "\n"
                      "Options:\n"
                      "  -y <channel>,             state channel, input\n"
                      "  -j <channel>,             reference channel, input\n"
                      "  -u <channel>,             reference channel, output\n"
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
    SNS_REQUIRE(cx.state_in, "Need state channel");
    SNS_REQUIRE(cx.ref_out, "Need ref_out channel");
    SNS_REQUIRE(cx.ref_in, "Need ref_in channel");

    cx.scenegraph = sns_scene_load();
    cx.n_q = aa_rx_sg_config_count(cx.scenegraph);

    /* Add allowed collisions between adjacent links. */
    for (aa_rx_frame_id i = 0; i < (aa_rx_frame_id)aa_rx_sg_frame_count(cx.scenegraph); i++) {
        aa_rx_frame_id parent = aa_rx_sg_frame_parent(cx.scenegraph, i);
        if (parent != AA_RX_FRAME_NONE && parent != AA_RX_FRAME_ROOT) {
            aa_rx_sg_allow_collision(cx.scenegraph, i, parent, true);
        }
    }
    /*
     * But wait, there's more!
     * TODO: unhardcode for the UR5+gripper.
     */
    aa_rx_sg_allow_collision_name(cx.scenegraph,
            "robotiq_85_right_finger_tip_joint", "robotiq_85_right_finger_joint", true);
    aa_rx_sg_allow_collision_name(cx.scenegraph,
            "robotiq_85_left_finger_tip_joint", "robotiq_85_left_finger_joint", true);
    aa_rx_sg_allow_collision_name(cx.scenegraph,
            "fts_fix", "robotiq_85_base_joint", true);
    aa_rx_sg_allow_collision_name(cx.scenegraph,
            "fts_fix", "ee_link-collision", true);

    for (aa_rx_frame_id i = 0; i < aa_rx_sg_frame_count(cx.scenegraph); i++) {
        printf("Frame %zu: %s (parent: %zu)\n",
            i,
            aa_rx_sg_frame_name(cx.scenegraph, i),
            aa_rx_sg_frame_parent(cx.scenegraph, i));
    }
    fflush(stdout);


    sns_motor_state_init(cx.scenegraph, cx.state_in, &cx.state_set, 0, NULL);

    /* Setup handlers for input channers. */
    size_t n_ref_in = sns_motor_channel_count(cx.ref_in);
    size_t n_state = sns_motor_channel_count(cx.state_in);
    cx.handlers = AA_NEW_AR(struct sns_evhandler, n_ref_in + n_state);
    sns_motor_ref_init(cx.scenegraph, cx.ref_in, &cx.ref_set_in, n_ref_in, cx.handlers);
    sns_motor_state_init(cx.scenegraph, cx.state_in, &cx.state_set, n_state, cx.handlers + n_ref_in);

    /* Setup channels */
    sns_motor_ref_init(cx.scenegraph, cx.ref_out, &cx.ref_set_out, 0, NULL);

    /* Start Event Loop */
    cx.period = aa_tm_sec2timespec( 1 / opt_frequency );
    sns_start();
    enum ach_status r =
        sns_evhandle( cx.handlers, n_ref_in + n_state,
                      &cx.period, periodic, &cx,
                      sns_sig_term_default,
                      ACH_EV_O_PERIODIC_TIMEOUT );
    SNS_REQUIRE( sns_cx.shutdown || (ACH_OK == r),
                 "Could not handle events: %s, %s\n",
                 ach_result_to_string(r),
                 strerror(errno) );
    /* Halt */
    AA_MEM_ZERO(cx.ref_set_out->u, cx.n_q);
    send_ref(&cx);

    sns_end();
    return 0;
}

enum ach_status periodic(void *cx_) {
    struct cx *cx = (struct cx *)cx_;
    struct timespec now;
    clock_gettime(ACH_DEFAULT_CLOCK, &now);

    sns_motor_ref_collate(&now, cx->ref_set_in);

    if(test_for_collisions(cx, now)) {
        send_ref(cx);
    }

    return (ACH_OK);
}

int test_for_collisions( struct cx *cx, struct timespec now) {
    /* TODO: Does this really make sense? */
    double dt = aa_tm_timespec2sec( aa_tm_sub(now, cx->t) );

    /* For now we use this: */
    dt = 0.01;
    cx->t = now;
    size_t n_q = cx->n_q;

    struct aa_ct_state *latest = sns_motor_state_get(cx->state_set);

    double *q_act = latest->q;

    /* Integrate (euler step) */
    double *q_act_copy;

    q_act_copy = (double*) aa_mem_region_local_alloc(sizeof(q_act[0]) * (size_t)n_q);
    memcpy(q_act_copy, q_act, sizeof(q_act[0]) * (size_t)n_q);
    /* TODO: what should the time step be? */
    cblas_daxpy((int)n_q, 6*dt, latest->dq, 1, q_act, 1);

    struct aa_rx_sg *scenegraph = cx->scenegraph;

    aa_rx_sg_cl_init(scenegraph);
    aa_rx_sg_init(scenegraph);
    struct aa_rx_cl *cl = aa_rx_cl_create(scenegraph);

    /* check for collisions */
    size_t n_tf = aa_rx_sg_frame_count(scenegraph);
    if(n_q != aa_rx_sg_config_count(scenegraph)) {
        printf("n_q not set correctly.");
    }

    double *TF = (double*) aa_mem_region_local_alloc(14*n_tf*sizeof(double));
    double *TF_rel = TF, *TF_abs = TF+7;

    /* TODO: consider aa_rx_sg_tf_update? */
    aa_rx_sg_tf(scenegraph, n_q, q_act, n_tf, TF_rel, 14, TF_abs, 14);

    struct aa_rx_cl_set * cl_set = aa_rx_cl_set_create(cx->scenegraph);

    if( aa_rx_cl_check(cl, n_tf, TF_abs, 14, cl_set)) {
        for (aa_rx_frame_id i = 0; i < (aa_rx_frame_id)aa_rx_sg_frame_count(cx->scenegraph); i++) {
            for (aa_rx_frame_id j = i; j < (aa_rx_frame_id)aa_rx_sg_frame_count(cx->scenegraph); j++) {
                if (aa_rx_cl_set_get(cl_set, i, j)) {
                    printf("Collision between %s and %s\n",
                            aa_rx_sg_frame_name(cx->scenegraph, i),
                            aa_rx_sg_frame_name(cx->scenegraph, j));
                }
            }
        }
        memcpy(latest->q, q_act_copy, sizeof(latest->q[0]) * (size_t)n_q);
        aa_mem_region_local_pop(TF);
        aa_mem_region_local_pop(q_act_copy);
        printf("collision found\n");
        fflush(stdout);
        return 1;
    }

    memcpy(latest->q, q_act_copy, sizeof(latest->q[0]) * (size_t)n_q);
    aa_mem_region_local_pop(TF);
    aa_mem_region_local_pop(q_act_copy);
    return 0;
}

void send_ref( struct cx *cx )
{
    struct timespec now;
    clock_gettime( ACH_DEFAULT_CLOCK, &now );

    for(size_t i = 0; i < cx->n_q; i++) {
        cx->ref_set_out->u[i] = 0.0;
        cx->ref_set_out->meta[i].mode = SNS_MOTOR_MODE_HALT;
    }
    sns_motor_ref_put(cx->ref_set_out, &now, 20e9); /* 20 second duration. */

    printf("halting robot\n");
    fflush(stdout);
}
