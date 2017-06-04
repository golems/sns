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

#include <amino/ct/state.h>

#include "sns.h"
#include "sns/joystick/gamepad.h"
#include "sns/event.h"
#include "sns/motor.h"


struct joint_ctrl {
    int button;
    struct sns_motor_map *map;

    struct joint_ctrl *next;

};

struct cx {
    struct aa_rx_sg *scenegraph;

    struct sns_motor_channel *ref_chan;
    struct sns_motor_channel *state_chan;

    struct sns_motor_ref_set *ref_set;
    struct sns_motor_state_set *state_set;


    aa_rx_frame_id end_effector;

    struct ach_channel js_channel;

     struct sns_evhandler *handlers;

    struct timespec period;

    struct aa_ct_state *state_act;


    struct joint_ctrl *joint_ctrl;

    double *q_ref;
    double *dq_ref;
};


static enum ach_status
handle_js( void *cx, void *msg, size_t msg_size );


static void
teleop( struct cx *cx, struct sns_msg_joystick *msg );

static void
halt( struct cx *cx );

void send_ref( struct cx *cx );

int
main(int argc, char **argv)
{
    struct cx cx = {0};
    /* parse options */
    double opt_frequency = 100;
    const char *opt_chan_joystick = NULL;
    const char *opt_end_effector = NULL;
    {
        int c = 0;
        while( (c = getopt( argc, argv, "u:y:j:e:Q:m:h?" SNS_OPTSTRING)) != -1 ) {
            switch(c) {
                SNS_OPTCASES_VERSION("sns-teleopd",
                                     "Copyright (c) 2015-2017, Rice University\n",
                                     "Neil T. Dantam")
            case 'u':
                sns_motor_channel_push( optarg, &cx.ref_chan );
                break;
            case 'y':
                sns_motor_channel_push( optarg, &cx.state_chan );
                break;
            case 'j':
                opt_chan_joystick = optarg;
                break;
            case 'e':
                opt_end_effector = optarg;
                break;
            case 'Q':
            {
                struct joint_ctrl *J = AA_NEW0(struct joint_ctrl);
                J->button = atoi(optarg);
                J->next = cx.joint_ctrl;
                cx.joint_ctrl = J;
                break;
            }
            case 'm':
                if( cx.joint_ctrl ) {
                    cx.joint_ctrl->map = sns_motor_map_parse(optarg);
                } else {
                    SNS_DIE("Need joint-space button parameter before joint map\n");
                }
                break;
            case '?':   /* help     */
            case 'h':
                puts( "Usage: sns-teleopd -j <joystick-channel> -u <ref-channel> -y <state-channel>\n"
                      "Teleop a robot.\n"
                      "\n"
                      "Options:\n"
                      "  -y <channel>,             state channel, input\n"
                      "  -j <channel>,             joystick channel, input\n"
                      "  -u <channel>,             reference channel, output\n"
                      "  -e <frame>,               end-effector frame\n"
                      "  -m <map>,                 motor map\n"
                      "  -Q <button>,              joint-space control button\n"
                      "  -V,                       Print program version\n"
                      "  -?,                       display this help and exit\n"
                      "\n"
                      "Examples:\n"
                      "  sns-teleopd -j joystick -y state -u ref\n"
                      "\n"
                      "  sns-teleopd -j joystick -y state -u ref -q 1 -m s0,s1,e0 -q 2 -m w0,w1,w2\n"
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
    SNS_REQUIRE(cx.ref_chan, "Need state channel");
    SNS_REQUIRE(cx.state_chan, "Need reference channel");
    SNS_REQUIRE(opt_chan_joystick, "Need joystick channel");

    /* Load Scene Plugin */
    cx.scenegraph = sns_scene_load();
    aa_rx_sg_init(cx.scenegraph);

    if( opt_end_effector ) {
        cx.end_effector = aa_rx_sg_frame_id(cx.scenegraph,opt_end_effector);
        SNS_REQUIRE( cx.end_effector > 0, "Invalid end-effector frame: `%s'", opt_end_effector );
    } else {
        cx.end_effector = AA_RX_FRAME_NONE;
    }

    /* Reference (output) */
    sns_motor_ref_init(cx.scenegraph,
                       cx.ref_chan, &cx.ref_set,
                       0, NULL );

    /* Input handlers */
    size_t n_handlers = 1 + sns_motor_channel_count(cx.state_chan);
    cx.handlers = AA_NEW_AR(struct sns_evhandler, n_handlers);


    /* Joystick */
    sns_chan_open( &cx.js_channel, opt_chan_joystick, NULL );
    cx.handlers[0].channel = &cx.js_channel;
    cx.handlers[0].context = &cx;
    cx.handlers[0].handler = handle_js;
    cx.handlers[0].ach_options = 0;
    for( struct joint_ctrl *J = cx.joint_ctrl; J; J = J->next ) {
        sns_motor_map_fill_id(cx.scenegraph,J->map);
    }

    /* State */
    sns_motor_state_init(cx.scenegraph,
                         cx.state_chan, &cx.state_set,
                         n_handlers - 1, cx.handlers + 1 );

    /* Start Event Loop */
    cx.period = aa_tm_sec2timespec( 1 / opt_frequency );
    sns_start();
    enum ach_status r =
        sns_evhandle( cx.handlers, n_handlers,
                      &cx.period, NULL, NULL,
                      sns_sig_term_default,
                      ACH_EV_O_PERIODIC_TIMEOUT );
    SNS_REQUIRE( sns_cx.shutdown || (ACH_OK == r),
                 "Could not handle events: %s, %s\n",
                 ach_result_to_string(r),
                 strerror(errno) );

    /* Halt */
    halt(&cx);

    sns_end();

    return 0;
}

enum ach_status handle_js( void *cx_, void *msg_, size_t msg_size )
{
    struct cx *cx = (struct cx*)cx_;
    struct sns_msg_joystick *msg = (struct sns_msg_joystick *)msg_;

    if( sns_msg_joystick_check_size(msg,msg_size) ) {
        /* Invalid Message */
        SNS_LOG(LOG_ERR, "Mismatched message size on joystick channel\n");
    } else {
        /* Process Message */
        teleop(cx,msg);
    }

    return ACH_OK;
}


void teleop( struct cx *cx, struct sns_msg_joystick *msg )
{
    for( size_t i = 0; i < cx->ref_set->n_q; i++ ) {
        cx->ref_set->u[i] = 0;
        cx->ref_set->meta[i].mode = SNS_MOTOR_MODE_VEL;
    }

    /* TODO: Workspace control */
    for( struct joint_ctrl *J = cx->joint_ctrl; J; J = J->next ) {
        if( msg->buttons & (uint64_t)(1 << J->button ) ) {
            sns_motor_map_in( J->map,
                              (size_t)msg->header.n, msg->axis,
                              cx->ref_set->u );

        }
    }

    struct timespec now;
    clock_gettime( ACH_DEFAULT_CLOCK, &now );
    sns_motor_ref_put( cx->ref_set, &now, 1e9 );
}

static void
halt( struct cx *cx )
{
    for( size_t i = 0; i < cx->ref_set->n_q; i++ ) {
        cx->ref_set->meta[i].mode = SNS_MOTOR_MODE_HALT;
        cx->ref_set->u[i]= 0;
    }

    struct timespec now;
    clock_gettime( ACH_DEFAULT_CLOCK, &now );
    sns_motor_ref_put( cx->ref_set, &now, 1e9 );
}
