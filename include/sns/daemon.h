/*
 * Copyright (c) 2013-2014, Georgia Tech Research Corporation
 * Copyright (c) 2015, Rice University
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
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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

/**
 * @file  daemon.h
 * @brief declarations for SNS daemons
 *
 * @author Neil T. Dantam
 */

#ifdef __cplusplus
extern "C" {
#endif


/**
 * When verbosity level is zero, print of this or greater severity
 */
#define SNS_LOG_LEVEL LOG_NOTICE

/**
 * Channel for log messages
 */
#define SNS_LOG_CHANNEL "sns-log"

/**
 * Default size for core dumps
 */
#define SNS_DEFAULT_CORE_SIZE (100 * (1<<20))


/**
 * Context struct for an SNS daemon.
 *
 * There is a single global instance of this struct.
 */
typedef struct sns_cx {
    int is_initialized;              ///< is the struct initialized?
    ach_channel_t chan_log;          ///< channel the gets log events
    pid_t pid;                       ///< pid of this process
    const char *ident;               ///< identifier for this daemon
    char host[SNS_HOSTNAME_LEN];     ///< hostname for this daemon
    struct timespec time_monotonic;  ///< monotonic time
    struct timespec time_real;       ///< real time
    volatile sig_atomic_t shutdown;  ///< set to true when system should shutdown
    int verbosity;                   ///< how much output to give.  Add SNS_LOG_LEVEL to get priority
    FILE *stderr;                    ///< file handler for printing log/error messages
} sns_cx_t;

/**
 * The global SNS daemon context
 */
extern struct sns_cx sns_cx;

/**
 * Priories for realtime proceses
 */
enum sns_prio {
    SOMATIC_PRIO_NONE    = 0,  ///< not realtime
    SNS_PRIO_UI          = 1,  ///< user interface
    SNS_PRIO_CONTROL     = 15, ///< controller
    SNS_PRIO_MOTOR       = 20, ///< motor
    SNS_PRIO_MAX         = 30  ///< highest realtime priority
};

/**
 * Initialize the SNS daemon.
 *
 * Call this function at when the daemon first starts and before
 * accessing the sns_cx struct.
 */
void sns_init( void );


struct sns_init_rt_opts {
    enum sns_prio prio;
};

/**
 * Make real-time
 */
void sns_init_rt( const struct sns_init_rt_opts *opts );

/**
 * Indicate that daemon is beginning its normal execuation.
 *
 * Call this function after the daemon successfully initialized, e.g.,
 * after it processes arguments and opens channels.
 *
 */
void sns_start( void );

/**
 * Destroy somatic daemon context struct.
 *
 * Call this function before your daemon exists.
 */
void sns_end( void );



/* -- Signal Handling -- */
/**
 * Install signal handlers for graceful shutdown.
 *
 * @post A signal handler is installed for every signal in sig.  The
 * signal handler will set the global sns_cx.shutdown to true and call
 * ach_cancel on every channel in chan.
 *
 * @param[in] chan a null-terminated array of ach channels.
 * @param[in] sig  a zero-terminated array of signals
 *
 * @see sns_sig_term_default
 *
 */
void sns_sigcancel( ach_channel_t **chan, const int sig[] );

/**
 * Signals which should terminate the process.
 *
 * You probably want to pass this to sns_sigcancel
 */
extern int sns_sig_term_default[];

/*********************/
/* Events and errors */
/*********************/

/**
 * Publish a log message
 *
 * @param[in] level a syslog priority
 * @param[in] code reserved
 * @param[in] fmt printf format string
 *
 *
 * @see SNS_LOG
 */
void sns_event( int level, int code, const char fmt[], ... )
#ifdef __GNUC__
    __attribute__((format(printf, 3, 4)))
#endif
    ;

/**
 * Terminate the process.
 */
void sns_die( void );

/**
 * Print an error message and terminate the process.
 */
#define SNS_DIE( ... ) { sns_event( LOG_CRIT, 0, __VA_ARGS__ ); sns_die(); }

/* Macros for to check and require conditions.
 * Other arguments are only evaluated if the test fails.
 */

/** Check whether condition is satisfied and log if false
 */
#define SNS_CHECK( test, priority, code, fmt, ... )                     \
    if( !(test) ) { sns_event( priority, code, fmt, __VA_ARGS__); }

/**
 * If test is false, then die
 *
 * @param test the condition to check
 *
 * The remainder of the arguments are printf format string and its
 * parameters.
 */
#define SNS_REQUIRE( test, ... )                \
    if( !(test) ) { SNS_DIE( __VA_ARGS__); }

/**
 * Evalute whether to log a message given priority.
 *
 * True if verbosity level indicates we should print message of given
 * priority.
 *
 */
#define SNS_LOG_PRIORITY( priority ) ((priority) <= SNS_LOG_LEVEL + sns_cx.verbosity)

/**
 * Publish a log message
 *
 * The arguments are a log priority level, a printf format string, and
 * the format string arguments.
 *
 * @param[in] priority a syslog logging priority
 */
#define SNS_LOG( priority, ... )                                        \
    if( SNS_LOG_PRIORITY(priority) ) { sns_event( priority, 0,  __VA_ARGS__); }

/********************/
/* Channel Handlers */
/********************/

/**
 * Open a channel or die
 *
 * @param[out] chan the channel handle struct
 * @param[in]  name the name of the channel to open
 * @param[in]  attr attributes for ach_open()
 */
void sns_chan_open( ach_channel_t *chan, const char *name,
                           ach_attr_t *attr );

/**
 * Close a channel
 */
void sns_chan_close( ach_channel_t *chan );


/********************/
/* Option Handling  */
/********************/

/**
 * Common arguments for SNS daemons
 */
#define SNS_OPTSTRING "vqV"

/**
 * The getopt() cases for common arguments to SNS daemons.
 */
#define SNS_OPTCASES                     \
    case 'v': sns_cx.verbosity++; break; \
    case 'q': sns_cx.verbosity--; break;


#define SNS_OPTCASES_VERSION(program, copyright, author)                \
    SNS_OPTCASES                                                        \
    case 'V':  {                                                        \
        puts( program " " PACKAGE_VERSION "\n"                          \
              "\n"                                                      \
              copyright                                                 \
              "This is free software; see the source for copying conditions.  There is NO\n" \
              "warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.\n" \
              "\n"                                                      \
              "Written by " author                                       \
              "\n"                                                      \
            );                                                          \
        exit(EXIT_SUCCESS);                                             \
    }

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
