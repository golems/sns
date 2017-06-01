/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */




#include "config.h"


#include <getopt.h>
#include <syslog.h>
#include <dlfcn.h>
#include "sns.h"


char *opt_channel = NULL;
enum sns_motor_mode opt_mode = SNS_MOTOR_MODE_POS;
sns_real_t *opt_u = NULL;
uint32_t n_opt_u = 0;
int opt_degrees = 0;

static void posarg( char *arg, int i ) {
    if( 0 == i ) {
        opt_channel = strdup(arg);
    } else {
        opt_u = (sns_real_t*)realloc( opt_u, (1+n_opt_u)*sizeof(opt_u[0]) );
        opt_u[n_opt_u++] = atof(arg);
    }
}

int main( int argc, char **argv ) {

    /*-- Parse Args -- */
    int i = 0;
    for( int c; -1 != (c = getopt(argc, argv, "?pdRHPD" SNS_OPTSTRING )); ) {
        switch(c) {
            SNS_OPTCASES_VERSION("snsref",
                                 "Copyright (c) 2013, Georgia Tech Research Corporation\n",
                                 "Neil T. Dantam")
        case 'p': opt_mode = SNS_MOTOR_MODE_POS; break;
        case 'd': opt_mode = SNS_MOTOR_MODE_VEL; break;
        case 'R': opt_mode = SNS_MOTOR_MODE_RESET; break;
        case 'H': opt_mode = SNS_MOTOR_MODE_HALT; break;
        case 'P': opt_mode = SNS_MOTOR_MODE_POS_OFFSET; break;
        case 'D': opt_degrees = 1; break;
        case '?':
            puts( "Usage: snsref [OPTIONS] channel x0 x1 ... xn\n"
                  "Send a motor referece command\n"
                  "\n"
                  "Options:\n"
                  "  -p                           Set positions (default)\n"
                  "  -d                           Set velocities\n"
                  "  -H                           Halt\n"
                  "  -P                           Set position offset\n"
                  "  -v,                          Make output more verbose\n"
                  "  -?,                          Give program help list\n"
                  "  -V,                          Print program version\n"
                  "\n"
                  "Report bugs to <ntd@gatech.edu>"
                );
            exit(EXIT_SUCCESS);
            break;
        default:
            posarg( optarg, i++ );
        }
    }

    while( optind < argc ) {
        posarg(argv[optind++], i++);
    }

    sns_init();

    SNS_REQUIRE( opt_channel, "snsref: missing channel.\nTry `snsref -?' for more information\n" );


    /*-- Open channel -- */
    ach_channel_t chan;
    sns_chan_open( &chan, opt_channel, NULL );
    {
        ach_channel_t *chans[] = {&chan, NULL};
        sns_sigcancel( chans, sns_sig_term_default );
    }

    /*-- Construct Message --*/

    struct sns_msg_motor_ref *msg = (struct sns_msg_motor_ref*)alloca( sns_msg_motor_ref_size_n(n_opt_u) );
    sns_msg_motor_ref_init( msg, n_opt_u);

    msg->mode = opt_mode;

    if( opt_degrees ) {
        for( size_t j = 0; j < n_opt_u; j ++ ) {
            msg->u[j] = opt_u[j] * M_PI/180.0;
        }
    } else {
        memcpy( msg->u, opt_u, n_opt_u*sizeof(msg->u[0]) );
    }

    struct timespec now;
    clock_gettime( ACH_DEFAULT_CLOCK, &now );
    sns_msg_set_time( &msg->header, &now, 1e9 ); /* 1 second duration */


    /*-- Send Message --*/
    if( SNS_LOG_PRIORITY(LOG_INFO) ) sns_msg_motor_ref_dump( stdout, msg );
    enum ach_status r = ach_put( &chan, msg, sns_msg_motor_ref_size(msg) );
    if( ACH_OK == r ) return 0;
    else {
        fprintf( stderr, "Failed to put message: %s\n", ach_result_to_string(r) );
    }
}
