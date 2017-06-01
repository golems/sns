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
#include <unistd.h>
#include "sns.h"
#include "sns/event.h"

static enum ach_status
handler ( void *context, void *msg, size_t msg_size );

char *opt_channel = NULL;
char *opt_type = NULL;
double opt_freq = 0;

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

    /*-- Parse Args -- */
    int i = 0;
    for( int c; -1 != (c = getopt(argc, argv, "?hHf:" SNS_OPTSTRING)); ) {
        switch(c) {
            SNS_OPTCASES_VERSION("snsdump",
                                 "Copyright (c) 2013, Georgia Tech Research Corporation\n",
                                 "Neil T. Dantam")
        case 'f':
            opt_freq = atof(optarg);
            break;
        case '?':   /* help     */
        case 'h':
        case 'H':
            puts( "Usage: snsdump [OPTIONS] channel message-type\n"
                  "Print SNS messages\n"
                  "\n"
                  "Options:\n"
                  "  -v,                          Make output more verbose\n"
                  "  -f FREQUENCY,                Sample at frequency\n"
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

    sns_init();

    SNS_REQUIRE( opt_channel, "snsdump: missing channel.\nTry `snsdump -H' for more information\n" );
    SNS_REQUIRE( opt_type, "snsdump: missing type.\nTry `snsdump -H' for more information\n" );


    SNS_LOG( LOG_INFO, "channel: %s\n", opt_channel );
    SNS_LOG( LOG_INFO, "type: %s\n", opt_type );
    SNS_LOG( LOG_INFO, "verbosity: %d\n", sns_cx.verbosity );

    /*-- Obtain Dump Function -- */
    sns_msg_dump_fun* fun =  (sns_msg_dump_fun*) sns_msg_plugin_symbol( opt_type, "sns_msg_dump" );
    SNS_REQUIRE( fun, "Couldn't link dump function symbol'\n");

    /*-- Open channel -- */
    ach_channel_t chan;
    sns_chan_open( &chan, opt_channel, NULL );

    /* setup handler */
    struct sns_evhandler handlers[1] = {
        {.channel = &chan,
         .context = fun,
         .ach_options = ACH_O_FIRST,
         .handler = handler
        }
    };

    /* run */
    enum ach_status r;
    r = sns_evhandle( handlers, sizeof( handlers ) / sizeof(handlers[0]),
                      NULL, NULL, NULL,
                      sns_sig_term_default, 0 );

    return r;
}

static enum ach_status
handler ( void *context, void *msg, size_t msg_size )
{
    (void)msg_size;
    sns_msg_dump_fun* fun  = (sns_msg_dump_fun*) context;
    (fun)(stdout, msg);
    return ACH_OK;
}
