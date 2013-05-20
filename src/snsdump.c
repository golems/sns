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
char *opt_type = NULL;

static void posarg( char *arg, int i ) {
    if( 0 == i ) {
        opt_channel = strdup(arg);
    } else if ( 1 == i ) {
        opt_type = strdup(arg);
    } else {
        fprintf(stderr, "Invalid arg: %s\n", arg);
        exit(EXIT_FAILURE);
    }
}

int main( int argc, char **argv ) {
    sns_init();

    /*-- Parse Args -- */
    int i = 0;
    for( int c; -1 != (c = getopt(argc, argv, "V?hH" SNS_OPTSTRING)); ) {
        switch(c) {
            SNS_OPTCASES
        case 'V':   /* version     */
            puts( "snsdump " PACKAGE_VERSION "\n"
                  "\n"
                  "Copyright (c) 2013, Georgia Tech Research Corporation\n"
                  "This is free software; see the source for copying conditions.  There is NO\n"
                  "warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.\n"
                  "\n"
                  "Written by Neil T. Dantam"
                );
            exit(EXIT_SUCCESS);
        case '?':   /* help     */
        case 'h':
        case 'H':
            puts( "Usage: snsdump [OPTIONS] channel message-type\n"
                  "Print SNS messages\n"
                  "\n"
                  "Options:\n"
                  "  -v,                          Make output more verbose\n"
                  "  -?,                          Give program help list\n"
                  "  -V,                          Print program version\n"
                  "\n"
                  "Examples:\n"
                  "  snsdump js_chan joystick     Dump 'joystick' messages from the 'js_chan' channel"
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

    SNS_REQUIRE( opt_channel, "snsdump: missing channel.\nTry `snsdump -H' for more information\n" );
    SNS_REQUIRE( opt_type, "snsdump: missing type.\nTry `snsdump -H' for more information\n" );


    SNS_LOG( LOG_INFO, "channel: %s\n", opt_channel );
    SNS_LOG( LOG_INFO, "type: %s\n", opt_type );
    SNS_LOG( LOG_INFO, "verbosity: %d\n", sns_cx.verbosity );

    /*-- Obtain Dump Function -- */
    sns_msg_dump_fun *fun =  (sns_msg_dump_fun*) sns_msg_plugin_symbol( opt_type, "sns_msg_dump" );
    SNS_REQUIRE( fun, "Couldn't link dump function symbol'\n");

    /*-- Open channel -- */
    ach_channel_t chan;
    sns_chan_open( &chan, opt_channel, NULL );
    {
        ach_channel_t *chans[] = {&chan, NULL};
        sns_sigcancel( chans, sns_sig_term_default );
    }

    /*-- Dump -- */
    sns_start();
    while(1) {
        void *buf;
        size_t frame_size;
        // get the frame
        ach_status_t r = sns_msg_local_get( &chan, &buf, &frame_size, NULL, ACH_O_WAIT );
        switch(r) {
        case ACH_MISSED_FRAME:
            SNS_LOG( LOG_WARNING, "Missed frame\n");
        case ACH_OK:
            (fun)( stdout, buf );
            break;
        case ACH_TIMEOUT:
            SNS_LOG( LOG_DEBUG+1, "timeout\n");
            break;
        case ACH_CANCELED:
            SNS_LOG( LOG_DEBUG, "Cancel received\n");
            exit(EXIT_SUCCESS);
        default:
            SNS_DIE(  "ach_get failed: %s\n", ach_result_to_string(r) );
        }
        aa_mem_region_local_release();
    }

    return 0;
}
