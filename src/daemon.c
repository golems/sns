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
        if (sigaction(SIGMSTART, &act, NULL) < 0) {
            perror ("sigaction");
            somatic_fail( "Couldn't install handler\n");
        }

        if (sigaction(SIGMSTOP, &act, NULL) < 0) {
            perror ("sigaction");
            somatic_fail( "Couldn't install handler\n");
        }

        if (sigaction(SIGMABORT, &act, NULL) < 0) {
            perror ("sigaction");
            somatic_fail( "Couldn't install handler\n");
        }

 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "sns.h"
#include <execinfo.h>
#include <unistd.h>
#include <stdio.h>
#include <syslog.h>
#include <signal.h>

struct sns_cx sns_cx = {0};


static void sighandler (int sig, siginfo_t *siginfo, void *context)
{
    (void) context;
    (void) siginfo;
    SNS_LOG( LOG_DEBUG, "Received Signal: %d, Sending PID: %ld, UID: %ld\n",
             sig, (long)siginfo->si_pid, (long)siginfo->si_uid);

    switch( sig) {
    case SIGINT:
    case SIGTERM:
        sns_cx.shutdown = 1;
    }
}

void sns_start( ) {
    // install signal handler
    {
        struct sigaction act;
        memset(&act, 0, sizeof(act));

        act.sa_sigaction = &sighandler;

        /* The SA_SIGINFO flag tells sigaction() to use the sa_sigaction field,
           not sa_handler. */
        act.sa_flags = SA_SIGINFO;

        int r;
        r = sigaction(SIGTERM, &act, NULL);
        SNS_REQUIRE( 0 == r, "sigaction failed: %s", strerror(errno) );

        r = sigaction(SIGINT, &act, NULL);
        SNS_REQUIRE( 0 == r, "sigaction failed: %s", strerror(errno) );
    }
}

/** Destroy somatic daemon context struct.

    Call this function before your daemon exists.
 */
void sns_end( ) {

}


static void sns_vevent( int priority, int code,
                        const char fmt[], va_list ap ) {
    (void) code;
    vfprintf(stderr, fmt, ap );

    // Maybe print a stack trace if something bad has happened
    switch( priority ) {
    case LOG_EMERG:
    case LOG_ALERT:
    case LOG_CRIT:
    case LOG_ERR:
    case LOG_WARNING:
        // print a backtrace to stderr if it's a tty
        if( isatty( STDERR_FILENO ) ) {
            fprintf(stderr,"--------STACK TRACE--------\n");
            static void *buffer[SNS_BACKTRACE_LEN];
            int n = backtrace( buffer, SNS_BACKTRACE_LEN );
            backtrace_symbols_fd( buffer, n, STDERR_FILENO );
            fprintf(stderr,"--------END STACK TRACE----\n");
        }
    }

}


void sns_event( int level, int code, const char fmt[], ... ) {
    va_list ap;
    va_start( ap, fmt );
    sns_vevent( level, code, fmt, ap );
    va_end( ap );
}

/** Terminates the process when things get really bad.*/
void sns_die( int code, const char fmt[], ... ) {
    // post event
    {
        va_list ap;
        va_start( ap, fmt );
        sns_vevent( LOG_EMERG, code, fmt, ap );
        va_end( ap );
    }

    // quit
    abort();
    exit(EXIT_FAILURE);
}



/** Opens a channel or dies if it can't */
void sns_chan_open( ach_channel_t *chan, const char *name,
                           ach_attr_t *attr ) {
    ach_status_t r = ach_open( chan, name, attr );
    SNS_REQUIRE( ACH_OK == r,
                 "Error opening channel `%s': %s\n",
                 name, ach_result_to_string(r));
    r =  ach_flush( chan );
    SNS_REQUIRE( ACH_OK == r,
                 "Error flushing channel `%s': %s\n",
                 name, ach_result_to_string(r));
}

/** Closes a channel */
void sns_chan_close( ach_channel_t *chan )
{
    ach_status_t r =  ach_close( chan );
    // not much to do if it fails, just log it
    SNS_CHECK( ACH_OK == r, LOG_ERR, 0,
               "Error closing channel: %s\n", ach_result_to_string(r)) ;
}
