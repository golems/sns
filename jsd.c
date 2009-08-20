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

/** \file jsd.c
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
        .name = "jsdev",
        .key = 'j',
        .arg = "device-num",
        .flags = 0,
        .doc = "specifies which device to use (default 0)"
    },
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


int main( int argc, char **argv ) {
    argp_parse (&argp, argc, argv, 0, NULL, NULL);
    if( opt_verbosity ) {
        fprintf(stderr, "Verbosity: %d\n", opt_verbosity);
        fprintf(stderr, "jsdev:     %d\n", opt_jsdev);
        fprintf(stderr, "channel:   %s\n", opt_ach_chan);
        fprintf(stderr,"-------\n");
    }

    // open ach channel
    ach_channel_t chan;
    { 
        int r = ach_open( &chan, opt_ach_chan, NULL );
        if( r != ACH_OK ) {
            fprintf(stderr, "Error opening ach channel: %s\n", ach_result_to_string( r ));
            exit(-1);
        }
    }

    // open js
    js_t *js = js_open( opt_jsdev );
    if (js == NULL) {
        perror("js_open");
        exit(-1);
    }


    // loop
    js_msg_t msg = {.axes = {0}, .buttons = 0};
    int n = js_msg_size( &msg );
    uint8_t *buf = alloca( n );
                        
    while(1) {
        assert(js);
        memset(&msg, 0, sizeof(msg) );
        memset(buf, 0, n );
        int i;
        int n_axes = sizeof(msg.axes)/sizeof(double);
        int n_buttons = sizeof(msg.buttons)*8;
        // poll js
        assert(js);
        memset(&msg, 0, sizeof(msg) );
        int status = js_poll_state( js );
        if (status != 0) {
            perror("js_poll_state");
            exit(-1);
        }
        
        // copy fields to msg
        for( i = 0; i < n_buttons; i++ ) {
            msg.buttons |= ( 0 != js->state.buttons[i] ) << i;
        }
        for( i = 0; i < n_axes; i++ ) {
            msg.axes[i] = js->state.axes[i];
        }

        // maybe print stuff
        if( opt_verbosity ) {
            fprintf(stderr,"axes:    [ ");
            for( i = 0; i < n_axes; i ++ ) {
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
        // send msg
        {
            js_msg_encode( buf, n, &msg );
            
            int r = ach_put( &chan, buf, n );
            if( r != ACH_OK ) {
                fprintf(stderr, "Error opening ach channel: %s\n", ach_result_to_string( r ));
                exit(-1);
            }
        }
    }

    //close 
    {
        int r = ach_close( &chan );
        if( r != ACH_OK ) {
            fprintf(stderr, "Error opening ach channel: %s\n", ach_result_to_string( r ));
        }
        js_close(js);
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
    case 0:
        break;
    }
    return 0;
}
