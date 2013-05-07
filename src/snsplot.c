/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2010-2011, Georgia Tech Research Corporation
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
#include "sns.h"



/*------------*/
/* PROTOTYPES */
/*------------*/

typedef struct {
    FILE* gnuplot;
    double *data;
    size_t n_samples;
    size_t n_each;
    size_t i;
    _Bool printed_header;
    char **labels;
} gnuplot_live_t;

typedef struct {
    ach_channel_t chan_in;
    gnuplot_live_t plot;
    size_t n_msg;
    sns_msg_plot_sample_fun *fun;
} cx_t;



/** Initialize the daemon */
static void init(cx_t *cx);
/** Main daemon run loop */
static void destroy(cx_t *cx);
/** Cleanup for exit */
static void run(cx_t *cx);
/** Update state */
static void update(cx_t *cx);

/** Get next msg */
static void next_msg(cx_t *cx, double **samples, char ***labels, size_t *n);

/** display plot */
static void plot(gnuplot_live_t *pl);

/* ------- */
/* GLOBALS */
/* ------- */

//static const char *opt_quantity = "position";
static double opt_range_min = -10;
static double opt_range_max = 10;
static size_t opt_samples = 100;
static double opt_frequency = 10;


static const char *opt_channel = "foo";
static const char *opt_type = "void";

/* ------- */
/* HELPERS */
/* ------- */

static void init(cx_t *cx) {
    sns_start();

    // open channel
    sns_chan_open( &cx->chan_in,
                   opt_channel, NULL );

    // open gnuplot
    cx->plot.gnuplot = popen("gnuplot -persist", "w");

    fprintf(cx->plot.gnuplot, "set title 'Channel: %s\n",
            opt_channel);
    fprintf(cx->plot.gnuplot, "set xlabel 'Time (s)'\n");
    //fprintf(cx->plot.gnuplot, "set ylabel '%s\n", opt_quantity);
    fprintf(cx->plot.gnuplot, "set yrange [%f:%f]\n", opt_range_min, opt_range_max);

    // get plugin
    cx->fun = (sns_msg_plot_sample_fun*) sns_msg_plugin_symbol( opt_type, "sns_msg_plot_sample" );
    SNS_REQUIRE( cx->fun, "Couldn't dlsym for %s\n", opt_type );

    // init struct
    char **labels;
    next_msg( cx, NULL, &labels, &cx->plot.n_each );
    cx->plot.n_samples = opt_samples;
    cx->plot.data = (double*)malloc(sizeof(double) * cx->plot.n_samples * cx->plot.n_each);
    cx->plot.labels = (char**)malloc(sizeof(char*) * cx->plot.n_each);
    for( size_t i = 0; i < cx->plot.n_each; i ++ ) {
        cx->plot.labels[i] = strdup(labels[i]);
    }
    cx->plot.printed_header = 0;
}


static void next_msg(cx_t *cx, double **samples, char ***labels, size_t *n) {
    void *buf;
    size_t frame_size;

    ach_status_t r = sns_msg_local_get( &cx->chan_in, &buf, &frame_size,
                                        NULL, ACH_O_WAIT );

    SNS_REQUIRE( (ACH_OK == r) || (ACH_MISSED_FRAME == r),
                 "Couldn't get frame: %s\n", ach_result_to_string(r) );
    if( ACH_MISSED_FRAME == r ) {
        fprintf(stderr, "missed frame\n");
    }
    cx->fun( buf, samples, labels, n );
}

static void update(cx_t *cx) {
    // get message
    size_t n;
    double *pbuf;
    next_msg(cx, &pbuf, NULL, &n);
    SNS_REQUIRE( n == cx->plot.n_each,
                 "Wrong sample size: %"PRIuPTR", wanted %"PRIuPTR"\n",
                 n, cx->plot.n_each );
    aa_fcpy( cx->plot.data + (cx->plot.i*cx->plot.n_each), pbuf,
             cx->plot.n_each );
    cx->plot.i = (cx->plot.i + 1) % cx->plot.n_samples;
}

static void run(cx_t *cx) {
    while(!sns_cx.shutdown) {
        update(cx);
        plot(&cx->plot);
        aa_mem_region_local_release();
        //usleep( (useconds_t) (1e6 / opt_frequency));
    }
}

void destroy(cx_t *cx) {
    // close channel
    sns_chan_close( &cx->chan_in );
    // close gnuplot
    fclose( cx->plot.gnuplot );
    // end daemon
    sns_end();
}


static void plot(gnuplot_live_t *pl) {
    // header
    if( ! pl->printed_header ) {
        pl->printed_header = 1;
        if( pl->labels )
            fprintf(pl->gnuplot, "plot '-' with points title '%s'",
                pl->labels[0]);
        else
            fprintf(pl->gnuplot, "plot '-' with points title '0'");
        for( size_t j = 1; j < pl->n_each; j++ ) {
            if( pl->labels )
                fprintf(pl->gnuplot, ", '-' with points title '%s'",
                    pl->labels[j]);
            else
                fprintf(pl->gnuplot, ", '-' with points title '%"PRIuPTR"'", j);
        }
        fprintf(pl->gnuplot, "\n");
    } else {
        fprintf(pl->gnuplot, "replot\n");
    }
    // data
    for (size_t j = 0; j < pl->n_each; j++ ) {
        for( size_t k = 0;  k < pl->n_samples; k ++ ) {
            size_t i = (k+pl->i) % pl->n_samples;
            size_t idx = j + i*pl->n_each;
            fprintf(pl->gnuplot, "%f, %f\n",
                    ((double)k/opt_frequency), (double)(pl->data)[idx] );
        }
        fprintf(pl->gnuplot, "e\n" );
    }
    fflush( pl->gnuplot );
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
    for( int c; -1 != (c = getopt(argc, argv, "V?hH0:1:" SNS_OPTSTRING)); ) {
        switch(c) {
            SNS_OPTCASES

        case '0':
            opt_range_min = atof(optarg);
            break;
        case '1':
            opt_range_max = atof(optarg);
            break;
        case 'V':   /* version     */
            puts( "snsplot " PACKAGE_VERSION "\n"
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
            puts( "Usage: snsplot [OPTIONS...] channel type\n"
                  "Shell tool for CANopen\n"
                  "\n"
                  "Options:\n"
                  "  -0 value,                    Minimum range value\n"
                  "  -1 value,                    Maximum range value\n"
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

    /*-- Run --*/
    init(&cx);
    run(&cx);
    destroy(&cx);

    return 0;
}
