/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
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
/** Author: Neil Dantam
 */

#include "config.h"

#include <inttypes.h>
#include <getopt.h>
#include <unistd.h>
#include "sns.h"

/*------------*/
/* PROTOTYPES */
/*------------*/

typedef struct {
    ach_channel_t chan;
    FILE *out;
    size_t n;
    sns_msg_plot_sample_fun fun;
} cx_t;

/** Initialize the daemon */
static void init(cx_t *cx);
/** Main daemon run loop */
static void destroy(cx_t *cx);
/** Cleanup for exit */
static void run(cx_t *cx);
/** Update state */
static void update(cx_t *cx, int header);

/* ------- */
/* GLOBALS */
/* ------- */

static const char *opt_channel = "foo";
static const char *opt_type = "void";
static const char *opt_out = NULL;

/* ------- */
/* HELPERS */
/* ------- */

static void init(cx_t *cx) {
    sns_start();

    // open channel
    sns_chan_open( &cx->chan,
                   opt_channel, NULL );

    // open output
    if( opt_out && 0 != strcmp(opt_out,"-") ) {
        cx->out = fopen(opt_out, "w");
        SNS_REQUIRE(NULL != cx->out, "Could not open file `%s'\n", opt_out);
    } else {
        cx->out = stdout;
    }

    // get plugin
    cx->fun = (sns_msg_plot_sample_fun*) sns_msg_plugin_symbol( opt_type, "sns_msg_plot_sample" );
    SNS_REQUIRE( cx->fun, "Couldn't dlsym for %s\n", opt_type );

    {
        ach_channel_t *chans[] = {&cx->chan, NULL};
        sns_sigcancel( chans, sns_sig_term_default );
    }

    // init struct
    update(cx,1);
}

static void update(cx_t *cx, int header) {
    // get message
    size_t n;
    double *sample;
    char **labels;

    struct sns_msg_header *buf;
    {
        size_t frame_size;
        ach_status_t r = sns_msg_local_get( &cx->chan, (void**)&buf, &frame_size,
                                            NULL, ACH_O_WAIT );

        if( ACH_CANCELED == r ) return;
        SNS_REQUIRE( (ACH_OK == r) || (ACH_MISSED_FRAME == r),
                     "Couldn't get frame: %s\n", ach_result_to_string(r) );
        if( ACH_MISSED_FRAME == r ) {
            fprintf(stderr, "missed frame\n");
        }
        cx->fun( buf, &sample, header ? &labels : NULL, &n );
    }

    sigset_t set;
    sigemptyset(&set);
    for( size_t i = 0; sns_sig_term_default[i]; i++ ) {
        sigaddset( &set, sns_sig_term_default[i] );
    }
    sigprocmask(SIG_BLOCK, &set, NULL);

    if( header ) {
        time_t t = time(NULL);
        char *time_str = ctime(&t);
        fprintf(cx->out,
                "# %s\n"
                "# channel: %s\n\n",
                time_str,
                opt_channel);
        fprintf(cx->out, "# time\t");
        for( size_t i = 0; i < n; i ++ )
            fprintf(cx->out,"%s%s", labels[i], (i == n-1) ? "\n\n" : "\t" );
        cx->n = n;
    }

    SNS_REQUIRE( n == cx->n,
                 "Wrong sample size: %"PRIuPTR", wanted %"PRIuPTR"\n",
                 n, cx->n );
    fprintf(cx->out,"%"PRId64".%09"PRIu32"\t", buf->sec, buf->nsec);
    aa_io_d_print( cx->out, n, sample, 1 );
    fflush(cx->out);

    sigprocmask(SIG_UNBLOCK, &set, NULL);
}

static void run(cx_t *cx) {
    while(!sns_cx.shutdown) {
        update(cx,0);
        aa_mem_region_local_release();
    }
}

void destroy(cx_t *cx) {
    fclose(cx->out);
    sns_chan_close( &cx->chan );
    sns_end();
}

/* ---- */
/* MAIN */
/* ---- */

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
    (void) argc; (void) argv;
    static cx_t cx;
    memset(&cx, 0, sizeof cx); // zero initialize

    /*-- Parse Options --*/
    int i = 0;
    for( int c; -1 != (c = getopt(argc, argv, "o:V?" SNS_OPTSTRING)); ) {
        switch(c) {
            SNS_OPTCASES
        case 'o':
            opt_out = optarg;
            break;
        case 'V':   /* version     */
            puts( "snsrec " PACKAGE_VERSION "\n"
                  "\n"
                  "Copyright (c) 2013, Georgia Tech Research Corporation\n"
                  "This is free software; see the source for copying conditions.  There is NO\n"
                  "warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.\n"
                  "\n"
                  "Written by Neil T. Dantam"
                );
            exit(EXIT_SUCCESS);
        case '?':   /* help     */
            puts( "Usage: snsrec [OPTIONS...] channel type\n"
                  "Record SNS messages\n"
                  "\n"
                  "Options:\n"
                  "  -o FILE,                     Output File\n"
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

    /*-- Run --*/
    init(&cx);
    run(&cx);
    destroy(&cx);

    return 0;
}
