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

#ifndef SNS_DAEMON_H
#define SNS_DAEMON_H

#ifdef __cplusplus
extern "C" {
#endif


// Print every message with lower priority than SNS_LOG_LEVEL + verbosity setting
#define SNS_LOG_LEVEL LOG_NOTICE

#define SNS_LOG_CHANNEL "sns-log"

#define SNS_DEFAULT_CORE_SIZE (100 * (1<<20))

typedef struct sns_cx {
    int is_initialized;              ///< is the struct initialized
    ach_channel_t chan_log;          ///< channel the gets other events
    pid_t pid;                       ///< pid of this process
    const char *ident;               ///< identifier for this daemon
    char host[SNS_HOSTNAME_LEN];      ///< hostname for this daemon
    struct timespec time_monotonic;  ///< monotonic time
    struct timespec time_real;       ///< real time
    volatile sig_atomic_t shutdown;  ///< set to true when system should shutdown
    int verbosity;                   ///< how much output to give.  Add SNS_LOG_LEVEL to get priority
    FILE *stderr;
} sns_cx_t;

extern struct sns_cx sns_cx;

/** Priority for realtime proceses */
enum sns_prio {
    SOMATIC_PRIO_NONE    = 0,  ///< not realtime
    SNS_PRIO_UI          = 1,  ///< user interface
    SNS_PRIO_CONTROL     = 15, ///< controller
    SNS_PRIO_MOTOR       = 20, ///< motor
    SNS_PRIO_MAX         = 30  ///< highest realtime priority
};

void sns_init( void );

void sns_start( void );

/** Destroy somatic daemon context struct.

    Call this function before your daemon exists.
 */
void sns_end( void );



/* -- Signal Handling -- */
/** When signal 'sig' is received, cancel waits on 'chan' */
void sns_sigcancel( ach_channel_t **chan, const int sig[] );

/* Signals which should terminate the process */
extern int sns_sig_term_default[];

/*********************/
/* Events and errors */
/*********************/

/** Publish an event message */
void sns_event( int level, int code, const char fmt[], ... )
#ifdef __GNUC__
    __attribute__((format(printf, 3, 4)))
#endif
    ;

/** Terminates the process when things get really bad.*/
void sns_die( void );

#define SNS_DIE( ... ) { sns_event( LOG_CRIT, 0, __VA_ARGS__ ); sns_die(); }

/* Macros for to check and require conditions.
 * Other arguments are only evaluated if the test fails.
 */

#define SNS_CHECK( test, priority, code, fmt, ... )                     \
    if( !(test) ) { sns_event( priority, code, fmt, __VA_ARGS__); }

#define SNS_REQUIRE( test, ... )                \
    if( !(test) ) { SNS_DIE( __VA_ARGS__); }


#define SNS_LOG_PRIORITY( priority ) ((priority) <= SNS_LOG_LEVEL + sns_cx.verbosity)

#define SNS_LOG( priority, ... )                                        \
    if( SNS_LOG_PRIORITY(priority) ) { sns_event( priority, 0,  __VA_ARGS__); }




/********************/
/* Channel Handlers */
/********************/

/** Opens a channel or dies if it can't */
void sns_chan_open( ach_channel_t *chan, const char *name,
                           ach_attr_t *attr );

/** Closes a channel */
void sns_chan_close( ach_channel_t *chan );


/********************/
/* Option Handling  */
/********************/
#define SNS_OPTSTRING "vq"

#define SNS_OPTCASES                     \
    case 'v': sns_cx.verbosity++; break; \
    case 'q': sns_cx.verbosity--; break;


#ifdef __cplusplus
}
#endif
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/* Local Variables:                          */
/* mode: c                                   */
/* c-basic-offset: 4                         */
/* indent-tabs-mode:  nil                    */
/* End:                                      */
#endif //SNS_DAEMON_H
