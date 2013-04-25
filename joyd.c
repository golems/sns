/* -*- mode: C; c-basic-offset: 4  -*- */
/* ex: set shiftwidth=4 expandtab: */
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

/** Author: jscholz, Neil Dantam
 */

#include <argp.h>
#include <syslog.h>
#include <sns.h>
#include <signal.h>
#include <unistd.h>
#include "js.h"


// context struct
typedef struct {
    ach_channel_t chan;
    timer_t timer;
    const char *opt_chan_name;
    uint8_t opt_jsdev;
    size_t opt_axis_cnt;
    size_t opt_button_cnt;
    struct timespec opt_period; // provide message at least every period
    js_t *js;
    struct sns_msg_joystick *msg;
} cx_t;


/* ---------- */
/* ARGP Junk  */
/* ---------- */
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
        .doc = "ach channel to use"
    },
    {
        .name = "axes",
        .key = 'a',
        .arg = "axis-count",
        .flags = 0,
        .doc = "number of joystick axes (default to 6)"
    },
    {
        .name = NULL,
        .key = 0,
        .arg = NULL,
        .flags = 0,
        .doc = NULL
    }
};


/// argp parsing function
static int parse_opt( int key, char *arg, struct argp_state *state);
/// argp program version
const char *argp_program_version = "jachd-0.0.1";
/// argp program arguments documentation
static char args_doc[] = "";
/// argp program doc line
static char doc[] = "reads from linux joystick and pushes out ach messages";
/// argp object
static struct argp argp = {options, parse_opt, args_doc, doc, NULL, NULL, NULL };


static int parse_opt( int key, char *arg, struct argp_state *state) {
    cx_t *cx = (cx_t*)state->input;
    switch(key) {
    case 'j':
        cx->opt_jsdev = (uint8_t)atoi(arg);
        break;
    case 'v':
        sns_cx.verbosity++;
        break;
    case 'c':
        cx->opt_chan_name = strdup( arg );
        break;
    case 'a':
        cx->opt_axis_cnt = (size_t)atoi(arg);
        break;
    case 0:
        break;
    }

    //somatic_d_argp_parse( key, arg, &cx->d_opts );

    return 0;
}

/* ------------- */
/* Function Defs */
/* ------------- */

/**
 * Block, waiting for a mouse event
 */
static int jach_read_to_msg( cx_t *cx )
{
    int status = js_poll_state( cx->js );
    if( !status ) {
        for( size_t i = 0; i < cx->msg->n && i < JS_AXIS_CNT; i++ ) {
            cx->msg->axis[i] = cx->js->state.axes[i];
        }

        cx->msg->buttons = 0;
        for( size_t i = 0; i < sizeof(cx->msg->buttons)*8 && i < JS_BUTTON_CNT; i++ ) {
            cx->msg->buttons |= ( (uint64_t)(cx->js->state.buttons[i] ? 1 : 0) << i );
        }

        if( SNS_LOG_PRIORITY( LOG_DEBUG ) && isatty(STDERR_FILENO) ) {
            sns_msg_joystick_dump( cx->msg );
        }
    }
    return status;
}

static void timer_handler(int sig) {
    // do nothing, the read will get EINTR
    (void)sig;
}

static int create_timer(cx_t *cx) {
    struct sigevent sev;
    struct itimerspec its;
    int r;
    struct sigaction sa;

    // setup sighandler
    memset(&sa,0,sizeof(sa));
    sa.sa_handler = timer_handler;
    sigemptyset(&sa.sa_mask);
    if( 0 != (r = sigaction(SIGALRM, &sa, NULL)) ) {
        syslog(LOG_WARNING, "failed sigaction: %s", strerror(errno));
        return r;
    }

    // create  timer
    sev.sigev_notify = SIGEV_SIGNAL;
    sev.sigev_signo = SIGALRM;
    sev.sigev_value.sival_ptr = &cx->timer;
    if( 0 != (r = timer_create(CLOCK_MONOTONIC, &sev, &cx->timer)) ) {
        syslog(LOG_WARNING, "failed timer_create: %s", strerror(errno));
        return r;
    }

    // start
    its.it_value = cx->opt_period;
    its.it_interval = cx->opt_period;
    if( 0 != (r = timer_settime(cx->timer, 0, &its, NULL)) ) {
        syslog(LOG_WARNING, "failed timer_settime: %s", strerror(errno));
        return r;
    }

    return 0;
}

static void jach_run( cx_t *cx ) {
    (void)cx;
    // main loop
    while (!sns_cx.shutdown) {
        int status = jach_read_to_msg( cx );
        if( !status || (EINTR == errno) ) {
            // ok, send the message
            //struct timespec now = aa_tm_now();
            //somatic_metadata_set_time_timespec( msg->meta, now );
            //SOMATIC_D_PUT(somatic__joystick, &cx->d, &cx->chan, msg );
        } else if( EAGAIN  == errno ) {
            // some system limit, try again
            sns_event( LOG_ERR, 0, "joystick EAGAIN" );
        } else {
            sns_die( 0, "joystick failure: %s", strerror(errno) );
        }
    }
}

/* ---- */
/* MAIN */
/* ---- */
int main( int argc, char **argv ) {
    static cx_t cx;
    memset(&cx, 0, sizeof(cx));

    // default options
    cx.opt_chan_name = "joystick";
    //cx.d_opts.ident = "jachd";
    //cx.d_opts.sched_rt = SOMATIC_D_SCHED_UI;
    cx.opt_jsdev = 0;
    cx.opt_axis_cnt = 6;
    cx.opt_button_cnt = 10;
    cx.opt_period = aa_tm_sec2timespec(1.0 / 30.0);

    argp_parse (&argp, argc, argv, 0, NULL, &cx);

    cx.msg = sns_msg_joystick_alloc( cx.opt_axis_cnt );

    //initialize
    sns_start();

    // Open joystick device
    cx.js = js_open( cx.opt_jsdev );
    SNS_REQUIRE( NULL != cx.js,
                 "joystick: %s", strerror(errno) );

    // open channel
    sns_chan_open( &cx.chan, cx.opt_chan_name, NULL );

    //Somatic__Joystick *msg = somatic_joystick_alloc(cx.opt_axis_cnt, cx.opt_button_cnt);

    SNS_LOG( LOG_DEBUG, "\n* JSD *\n");
    SNS_LOG( LOG_DEBUG, "jsdev:        %d\n", cx.opt_jsdev);
    SNS_LOG( LOG_DEBUG, "js channel:      %s\n", cx.opt_chan_name);
    SNS_LOG( LOG_DEBUG, "js period:       %fs\n", aa_tm_timespec2sec(cx.opt_period));
    //fprintf(stderr, "message size: %"PRIuPTR"\n", somatic__joystick__get_packed_size(msg) );

    // This gives read() an EINTR when the timer expires,
    // so we can republish the message instead of blocking on the joystick read.
    // Note that blocks on ach channels take an expiration time directly, so
    // no timer would be needed for waits there.
    if( create_timer(&cx) ) {
        SNS_LOG(LOG_WARNING, "Couldn't create timer, joyd will block: %s", strerror(errno));
    }

    // run
    jach_run( &cx );

    // Cleanup:
    //somatic_joystick_free(msg);
    sns_chan_close( &cx.chan );
    sns_end();

    js_close(cx.js);

    return 0;
}
