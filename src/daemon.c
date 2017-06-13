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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif /* HAVE_CONFIG_H */

#include "sns.h"
#include <execinfo.h>
#include <unistd.h>
#include <stdio.h>
#include <syslog.h>
#include <signal.h>
#include <sys/resource.h>
#include <sched.h>
#include <sys/mman.h>


#include <amino.h>
#include <amino/rx/rxtype.h>
#include <amino/rx/scenegraph.h>
#include <amino/rx/scene_plugin.h>

struct sns_cx sns_cx = {0};

void sns_set_ident( const char * ident) {
    sns_cx.ident = ident;
}

/* Redirection of stderr to sns log */
#ifdef _GNU_SOURCE
static ssize_t redir_write( void *cookie, const char *data, size_t n ) {
    (void)cookie;
    assert( 1 == sizeof(char) );
    char buf[n+1];
    memcpy(buf, data, n);
    buf[n] = '\0';
    SNS_LOG( LOG_ERR, "%s", buf );
    return (ssize_t)n;
}

static ssize_t redir_read(void *c, char *buf, size_t size) {
    (void)c; (void)buf; (void)size;
    return 0;
}
static int redir_close(void *c ) {
    (void)c; return 0;
}

static int redir_seek(void *c, off64_t *offset, int whence) {
    (void)c; (void)offset; (void)whence;
    return 0;
}

static cookie_io_functions_t redir_vtab = {
    .read = &redir_read,
    .write = &redir_write,
    .seek = &redir_seek,
    .close = &redir_close
};

static void redir_stderr (void) {
    FILE *tmp = fopencookie( NULL, "w", redir_vtab );
    if( NULL == tmp ) {
        SNS_LOG( LOG_ERR, "Couldn't fopencookie for stderr\n" );
        return;
    }
    if( setvbuf(tmp, NULL, _IONBF, 0) ) {
        SNS_LOG( LOG_ERR, "Couldn't set buffering for cookie'ed stderr\n");
        return;
    }
    fclose(stderr);
    close(STDERR_FILENO);
    stderr = tmp;
}
#endif /*_GNU_SOURCE */



void sns_init( void ) {
    char *ptr = NULL;

    /* pid */
    sns_cx.pid = getpid();

    /* hostname */
    gethostname( sns_cx.host, SNS_HOSTNAME_LEN );

    /* log channel */
    {
        ach_status_t r = ach_open( &sns_cx.chan_log, SNS_LOG_CHANNEL, NULL );
        SNS_REQUIRE( ACH_OK == r,
                     "Error opening log channel: `%s'.  Is sns started?\n",
                     ach_result_to_string(r));
    }

    /* Check where to send error messages */
    if( isatty(STDERR_FILENO) ) {
        sns_cx.stderr = stderr;
    } else {
        sns_cx.stderr = NULL;

    /* Maybe redirect stderr to sns log */
    /* If any libraries write to stderr now, we can capture their
     * output in syslog.  This is easier to check than having
     * separate output files for each process. */
#ifdef _GNU_SOURCE
    redir_stderr();
#endif /*_GNU_SOURCE */
    }

    /* default signal handlers */
    sns_sigcancel( NULL, sns_sig_term_default );

    /* Ident */
    if( NULL != (ptr = getenv("SNS_IDENT")) ) {
        sns_set_ident(ptr);
    } else {
        sns_set_ident("sns");
    }

    /* cd to tmp dir */
    /* This is where we may dump core */
    if( NULL != (ptr = getenv("SNS_TMPDIR")) ) {
        if( chdir(ptr) ) {
            SNS_LOG(LOG_ERR, "Couldn't chdir: %s\n", strerror(errno));
        }
    }

    /* set limits */
    {
        struct rlimit lim;
        /* get max core size */
        if( getrlimit( RLIMIT_CORE, &lim ) ) {
            SNS_LOG(LOG_ERR, "Couldn't get RLIMIT_CORE: %s", strerror(errno));
        } else {
            /* set core size */
            lim.rlim_cur = (lim.rlim_max < SNS_DEFAULT_CORE_SIZE) ?
                lim.rlim_max : SNS_DEFAULT_CORE_SIZE;
            if( setrlimit( RLIMIT_CORE, &lim ) ) {
                SNS_LOG(LOG_ERR, "Couldn't get RLIMIT_CORE: %s", strerror(errno));
            }
        }
    }

    sns_cx.is_initialized = 1;
}


void sns_init_rt( const struct sns_init_rt_opts *opts )
{
    sns_init();

    // set scheduling policy
    {
        int max = sched_get_priority_max( SCHED_RR );
        int min = sched_get_priority_min( SCHED_RR );
        if( max >= 0 && min >= 0 ) {
            int pri = opts->prio;
            if( pri > 0 && pri >= min ) {
                struct sched_param sp;
                /* 32 is max portable priority*/
                if( pri > max ) {
                    syslog(LOG_WARNING, "Requested priority %d exceeds max %d",
                           pri, max);
                    pri = max;
                }
                sp.sched_priority = pri;
                if( sched_setscheduler( 0, SCHED_RR, &sp) < 0 ) {
                    syslog(LOG_ERR, "Couldn't set scheduling priority to %d: %s\n",
                           pri, strerror(errno) );
                }
            }
        } else {
            syslog(LOG_ERR, "Couldn't get scheduling priorities: %d, %d, %s\n",
                   max, min, strerror(errno) );
        }
    }

    // lock memory, can't swap to disk

    char prefault[1024*256];
    memset( prefault,0,sizeof(prefault) );
    if( mlockall( MCL_CURRENT | MCL_FUTURE ) ) {
        syslog( LOG_ERR, "Couldn't lock pages in memory: %s",
                strerror(errno) );
    }

}


static ach_channel_t **cancel_chans = NULL;

static void sighandler_cancel ( int sig ) {
    (void)sig;
    if( SNS_LOG_PRIORITY(LOG_DEBUG) ) {
        const char buf[] = "Received signal to cancel\n";
        write(STDERR_FILENO, buf, sizeof(buf)); /* write is async safe, xprintf family is not */
    }
    /* mark shutdown */
    sns_cx.shutdown = 1;
    /* cancel channel operations */
    if(cancel_chans) {
        for( size_t i = 0; NULL != cancel_chans[i]; i ++ ) {
            enum ach_status r = ach_cancel(cancel_chans[i], NULL);
            if( ACH_OK != r ) {
                static const char err[] = "Failure in sighandler_cancel\n";
                write(STDERR_FILENO, err, sizeof(err) );
                /* exit(EXIT_FAILURE); */
                /* exit could deadlock */
                /* hope we catch the shutdown somewhere */
            }
        }
    }
}

void sns_sigcancel( ach_channel_t **chan, const int *sig ) {
    /* register channel */
    if(chan) {
        size_t i;
        for( i = 0; NULL != chan[i]; i ++ );
        i++; // one more for the null
        cancel_chans = (ach_channel_t**)realloc( cancel_chans, i * sizeof(cancel_chans[0]) );
        memcpy( cancel_chans, chan, i*sizeof(chan[0]) );
    }

    /* setup signal handler */
    for( size_t i = 0; sig[i]; i ++ ) {
        struct sigaction act;
        memset(&act, 0, sizeof(act));
        act.sa_handler = &sighandler_cancel;
        if( sigaction(sig[i], &act, NULL) ) {
            SNS_DIE( "Could not install signal handler\n");
        }
    }
}

int sns_sig_term_default[] = {
    SIGHUP, SIGTERM, SIGINT, SIGUSR1, SIGUSR2,
    0 };


/* static void ach_sigdummy(int sig) { */
/*     (void)sig; */
/* } */


/* static void sig_mask( const int *sig, sigset_t *mask ) { */
/*     if( sigemptyset(mask) ) sns_die(0,"sigemptyset failed: %s\n", strerror(errno)); */
/*     size_t i; */
/*     for( i = 0; sig[i]; i ++ ) { */
/*         if( sigaddset(mask, sig[i]) ) { */
/*             sns_die(0, "sigaddset of %s (%d) failed: %s\n", */
/*                     strsignal(sig[i]), sig[i], strerror(errno) ); */
/*         } */
/*     } */
/* } */

/* void sns_sigblock( const int *sig ) { */
/*     /\* Block Signal *\/ */
/*     { */
/*         sigset_t blockmask; */
/*         sig_mask( sig, &blockmask ); */
/*         if( sigprocmask(SIG_BLOCK, &blockmask, NULL) ) { */
/*             sns_die(0, "sigprocmask failed: %s\n", strerror(errno) ); */
/*         } */
/*     } */
/*     /\* Install Dummy Handler *\/ */
/*     size_t i; */
/*     for( i = 0; sig[i]; i ++ ) { */
/*         struct sigaction act; */
/*         memset( &act, 0, sizeof(act) ); */
/*         act.sa_handler = &ach_sigdummy; */
/*         if (sigaction(sig[i], &act, NULL) < 0) { */
/*             sns_die( 0, "Couldn't install signal handler: %s", strerror(errno) ); */
/*         } */
/*     } */
/* } */


/* static void sighandler (int sig, siginfo_t *siginfo, void *context) */
/* { */
/*     (void) context; */
/*     (void) siginfo; */
/*     SNS_LOG( LOG_DEBUG, "Received Signal: %d, Sending PID: %ld, UID: %ld\n", */
/*              sig, (long)siginfo->si_pid, (long)siginfo->si_uid); */

/*     switch( sig) { */
/*     case SIGINT: */
/*     case SIGTERM: */
/*         sns_cx.shutdown = 1; */
/*     } */
/* } */

void sns_start( ) {

    // tell achcop parent we're starting
    if( getenv("ACHCOP") ) {
        kill(getppid(), SIGUSR1);
    }

    /* // install signal handler */
    /* { */
    /*     struct sigaction act; */
    /*     memset(&act, 0, sizeof(act)); */

    /*     act.sa_sigaction = &sighandler; */

    /*     /\* The SA_SIGINFO flag tells sigaction() to use the sa_sigaction field, */
    /*        not sa_handler. *\/ */
    /*     act.sa_flags = SA_SIGINFO; */

    /*     int r; */
    /*     r = sigaction(SIGTERM, &act, NULL); */
    /*     SNS_REQUIRE( 0 == r, "sigaction failed: %s", strerror(errno) ); */

    /*     r = sigaction(SIGINT, &act, NULL); */
    /*     SNS_REQUIRE( 0 == r, "sigaction failed: %s", strerror(errno) ); */
    /* } */
}

/** Destroy somatic daemon context struct.

    Call this function before your daemon exists.
 */
void sns_end( ) {

}



void sns_event( int level, int code, const char fmt[], ... ) {
    (void) code;
    /* maybe stderr */
    FILE *err = sns_cx.is_initialized ? sns_cx.stderr : stderr ;
    if( err ) {
        va_list ap;
        va_start( ap, fmt );
        vfprintf(err, fmt, ap );
        va_end( ap );
        /* Print a stack trace if something bad has happened */
        switch( level ) {
        case LOG_EMERG:
        case LOG_ALERT:
        case LOG_CRIT:
        case LOG_ERR:
        case LOG_WARNING:
            // print a backtrace to stderr if it's a tty
            fprintf(err,"\n--------STACK TRACE--------\n");
            static void *buffer[SNS_BACKTRACE_LEN];
            int n = backtrace( buffer, SNS_BACKTRACE_LEN );
            backtrace_symbols_fd( buffer, n, fileno(err) );
            fprintf(err,"--------END STACK TRACE----\n\n");
        }
        return;
    }

    /* else not stderr */

    int size;
    /* get size */
    {
        va_list ap;
        va_start( ap, fmt );
        size = vsnprintf( NULL, 0, fmt, ap );
        va_end( ap );
    }

    /* make message */
    uint32_t n_str = (uint32_t)size + 1 + 1; /* size excludes null and maybe add trailing newline */
    size_t n_msg = sns_msg_log_size_n(n_str);
    sns_msg_log_t *msg  = (sns_msg_log_t*)alloca(n_msg);
    msg->header.n = n_str;
    msg->priority = level;
    sns_msg_header_fill( &msg->header );
    {
        va_list ap;
        va_start( ap, fmt );
        size = vsnprintf( msg->text, msg->header.n, fmt, ap );
        va_end( ap );
    }
    if( '\n' != msg->text[size-1] ) {
        msg->text[size] = '\n';
        msg->text[size+1] = '\0';
    }

    /* send message */
    enum ach_status r = ach_put( &sns_cx.chan_log, msg, n_msg );
    if( ACH_OK != r ) {
        syslog(LOG_ALERT, "Could not put log message: %s\n", ach_result_to_string(r));
        syslog( level, "%s", msg->text );
    }

    /* if( isatty(STDERR_FILENO) ){ */
    /*     fprintf( stderr, "%s", msg->text ); */
    /* } */
}

/** Terminates the process when things get really bad.*/
void sns_die( ) {
    /* tell achcop parent we're broken */
    if( getenv("ACHCOP") ) {
        kill(getppid(), SIGUSR2);
    }

    /* quit */
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

AA_API struct aa_rx_sg*
sns_scene_load(void)
{
    const char *plugin = getenv("SNS_SCENE_PLUGIN");
    const char *name = getenv("SNS_SCENE_NAME");

    SNS_REQUIRE( NULL != plugin, "SNS_SCENE_PLUGIN not defined");
    SNS_REQUIRE( NULL != name, "SNS_SCENE_NAME not defined");

    struct aa_rx_sg *scenegraph = aa_rx_dl_sg(plugin, name, NULL);
    SNS_REQUIRE( NULL != scenegraph, "Could not load scene plugin");

    aa_rx_sg_init(scenegraph);
    return scenegraph;
}
