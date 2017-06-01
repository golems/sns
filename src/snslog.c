/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
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


int opt_priority = LOG_INFO;
char *opt_message = NULL;

static void posarg( char *arg, int i ) {
    if( 0 == i ) {
        opt_message = strdup(arg);
    } else {
        opt_message = (char*)realloc(opt_message, strlen(opt_message) + strlen(arg) + 2);
        strcat(opt_message, " ");
        strcat(opt_message, arg);
    }
}

int main( int argc, char **argv ) {
    sns_init();

    /*-- Parse Args -- */
    int i = 0;
    for( int c; -1 != (c = getopt(argc, argv, "012345678V?hH" SNS_OPTSTRING)); ) {
        switch(c) {
            SNS_OPTCASES
        case 'V':   /* version     */
            puts( "snslog " PACKAGE_VERSION "\n"
                  "\n"
                  "Copyright (c) 2013, Georgia Tech Research Corporation\n"
                  "This is free software; see the source for copying conditions.  There is NO\n"
                  "warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.\n"
                  "\n"
                  "Written by Neil T. Dantam"
                );
            exit(EXIT_SUCCESS);
        case '0':
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
        case '6':
        case '7':
        case '8':
        {
            char buf[] = {(char)c, '\0'};
            opt_priority = atoi(buf);
        }
            break;
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
    if( NULL == opt_message ) opt_message = strdup("");

    /*-- Create Message -- */
    size_t n_str = strlen(opt_message);
    size_t n_msg = sns_msg_log_size_n((uint32_t)n_str);
    sns_msg_log_t *msg  = (sns_msg_log_t*)aa_mem_region_local_alloc(n_msg);
    memset(msg,0,n_msg);
    sns_msg_header_fill( &msg->header );
    strcpy(msg->text, opt_message);
    msg->priority = opt_priority;
    msg->header.n = n_str;


    /*-- Put Message -- */
    enum ach_status r = ach_put( &sns_cx.chan_log, msg, n_msg );
    if( ACH_OK != r ) {
        fprintf(stderr, "Could not put message: %s\n", ach_result_to_string(r));
        exit(EXIT_FAILURE);
    }

    return 0;
}
