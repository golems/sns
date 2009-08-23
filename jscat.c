/* -*- mode: C; c-basic-offset: 4  -*- */
/*
 * Copyright (c) 2008, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \file ach.c
 *  \author Neil T. Dantam
 */

#include <assert.h>
#include <time.h>
#include <stdint.h>
#include <string.h>
#include <argp.h>
#include <stdio.h>
#include <stdlib.h>
#include <ach.h>
#include <genmsg.h>
#include <js/js.h>
#include <js/js_msg.h>

static struct argp_option options[] = {
    {
        .name = "verbose",
        .key = 'v',
        .arg = NULL,
        .flags = 0,
        .doc = "Causes verbose output"
    },
    {
        .name = "channel",
        .key = 'c',
        .arg = "channel",
        .flags = 0,
        .doc = "ach channel to use (default \"js\")"
    },
    {
        .name = "axes",
        .key = 'a',
        .arg = "axis_cnt",
        .flags = 0,
        .doc = "number of axes"
    },
    {
        .name = NULL,
        .key = 0,
        .arg = NULL,
        .flags = 0,
        .doc = NULL
    }, 
};



/// argp parsing function
static int parse_opt( int key, char *arg, struct argp_state *state);
/// argp program version
const char *argp_program_version = "jsd-0.0.1";
/// argp program arguments documention
static char args_doc[] = "";
/// argp program doc line
static char doc[] = "reads from linux joystick and pushes out ach messages";
/// argp object
static struct argp argp = {options, parse_opt, args_doc, doc, NULL, NULL, NULL };


static int opt_jsdev = 0;
static int opt_verbosity = 0;
static int opt_freq_hz = 10;
static char *opt_ach_chan = "js";
static int opt_axis_cnt = 6;


int main( int argc, char **argv ) {
    argp_parse (&argp, argc, argv, 0, NULL, NULL);
    if( opt_verbosity ) {
        fprintf(stderr, "channel:   %s\n", opt_ach_chan);
    }
    fprintf(stderr,"-------\n");

    // open ach channel
    ach_channel_t chan;
    { 
        int r = ach_open( &chan, opt_ach_chan, NULL );
        if( r != ACH_OK ) {
            fprintf(stderr, "Error opening ach channel: %s\n", ach_result_to_string( r ));
            exit(-1);
        }
    }

    // loop
    
    js_msg_t msg;
    js_msg_init(&msg);
    double axes[opt_axis_cnt];
    msg.axes = axes;
    msg.axes_max = sizeof(axes)/sizeof(double);
    msg.axes_fill = opt_axis_cnt;
    size_t n = js_msg_size( &msg );
    uint8_t buf[n];

    while(1) {
        int i, r;
        int n_axes = sizeof(axes)/sizeof(double);
        int n_buttons = sizeof(msg.buttons)*8;
        memset(msg.axes, 0, sizeof(double) * n_axes );
        size_t frame_size;

        r = ach_wait_next(&chan, buf, n,  &frame_size, NULL );
        if( r != ACH_OK && r != ACH_MISSED_FRAME )  {
            fprintf(stderr, "Error reading ach frame: %s\n", ach_result_to_string( r ));
            exit(-1);
        }
        if( frame_size != n )  {
            fprintf(stderr, "wrong frame size: %d, should be %d\n", frame_size, n );
            exit(-1);
        }
        if( opt_verbosity ) {
            fprintf(stderr, "typecode: ");
            for( i = 0; i < 16; i ++ )
                fprintf(stderr,"%d ", buf[i] );
            fprintf(stderr, "\n");
        }

        r = js_msg_decode(&msg, buf, frame_size );
        assert( n >= r );

        // maybe print stuff
        
        fprintf(stderr,"axes:    [ ");
        for( i = 0; i < msg.axes_fill; i ++ ) {
            double axis = msg.axes[i];
            fprintf(stderr, "%s%5.4f ", axis < 0 ? "": " ", axis );
        }
        fprintf(stderr,"]\n");
        fprintf(stderr,"buttons: [ ");
        for( i = 0; i < n_buttons; i ++ ) {
            //fprintf(stderr, "%d ", js->state.buttons[i] );
            fprintf(stderr, "%d ", (msg.buttons >> i) & 1 );
        }
        fprintf(stderr,"]\n");
        fprintf(stderr,"-------\n");
        
    }

    //close 
    {
        int r = ach_close( &chan );
        if( r != ACH_OK ) {
            fprintf(stderr, "Error opening ach channel: %s\n", ach_result_to_string( r ));
        }
    }
    
}

static int parse_opt( int key, char *arg, struct argp_state *state) {
    (void) state; // ignore unused parameter
    switch(key) {
    case 'j':
        opt_jsdev = atoi(arg);
        break;
    case 'v':
        opt_verbosity++;
        break;
    case 'c':
        opt_ach_chan = strdup( arg );
        break;
    case 'a':
        opt_axis_cnt = atoi( arg );
        break;
    case 0:
        break;
    }
    return 0;
}
