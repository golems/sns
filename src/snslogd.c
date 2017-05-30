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

#include "config.h"
#include "sns.h"
#include <unistd.h>
#include <syslog.h>
#include <fcntl.h>
#include <inttypes.h>



/*------------*/
/* PROTOTYPES */
/*------------*/

/* Make a beep */
static void beep( int fd, int priority );
/* Process a message */
static void process( int fd_beep, sns_msg_log_t *msg, size_t frame_size );


static const char *opt_console = "/dev/tty0";

/* ---- */
/* MAIN */
/* ---- */
int main( int argc, char **argv ) {
    (void) argc; (void) argv;

    {
        for( int c; -1 != (c = getopt(argc, argv, "?" SNS_OPTSTRING)); ) {
            switch(c) {
                SNS_OPTCASES_VERSION("snslogd",
                                     "Copyright (c) 2013, Georgia Tech Research Corporation\n",
                                     "Neil T. Dantam")
            case '?':
                puts( "Usage: snslogd\n"
                      "Handle logging for SNS daemons\n");
                exit(EXIT_SUCCESS);
            default:
                SNS_DIE("Unknown Option: `%c'\n", c);
            }
        }
    }

    sns_cx.verbosity = 10;
    sns_init();
    openlog("sns", isatty(STDERR_FILENO) ? LOG_PERROR : 0, LOG_USER);

    int fd_beep = open(opt_console, O_WRONLY);

    // open console
    if( fd_beep < 0 ) {
        syslog(LOG_ERR, "couldn't open `%s' to beep: %s\n",
               opt_console, strerror(errno));
    }

    // cancel handlers
    {
        ach_channel_t *chans[] = {&sns_cx.chan_log, NULL};
        sns_sigcancel( chans, sns_sig_term_default );
    }

    sns_start();

    while( !sns_cx.shutdown ) {
        size_t frame_size;
        void *buf;
        ach_status_t r = sns_msg_local_get( &sns_cx.chan_log, &buf, &frame_size, NULL, ACH_O_WAIT );
        switch(r) {
        case ACH_MISSED_FRAME:
            syslog(LOG_WARNING, "missed log frame: %s", ach_result_to_string(r));
        case ACH_OK:
            /* process message */
            process( fd_beep, (sns_msg_log_t*)buf, frame_size );
            break;
        case ACH_CANCELED:
            break;
        default:
            syslog(LOG_ALERT, "ach_get failed: %s", ach_result_to_string(r));
        }
        aa_mem_region_local_release();
    }

    close(fd_beep);
    sns_end();

    return 0;
}

static void process( int fd_beep, sns_msg_log_t *msg, size_t frame_size ) {
    if( SNS_MSG_CHECK_SIZE(log, msg, frame_size) ) {
        beep(fd_beep, LOG_ERR);
        syslog(LOG_ERR, "Invalid message size: %"PRIuPTR, frame_size);
        return;
    } /* else, log the message */

    /* Log it */
    syslog( msg->priority, "[%s(%"PRIu64")@%s] %s",
            sns_str_nullterm(msg->header.ident, sizeof(msg->header.from_host)),
            msg->header.from_pid,
            sns_str_nullterm(msg->header.from_host, sizeof(msg->header.from_host)),
            sns_str_nullterm(msg->text, msg->header.n) );

    /* Maybe beep */
    if( fd_beep >= 0 ) {
        beep(fd_beep, msg->priority);
    }
}

static void beep( int fd, int priority ) {
    switch(priority) {
    case LOG_EMERG:
        sns_beep(fd, SNS_BEEP_NOTE_A7, 3);
        break;
    case LOG_ALERT:
        sns_beep(fd, SNS_BEEP_NOTE_A6, 2);
        break;
    case LOG_CRIT:
        sns_beep(fd, SNS_BEEP_NOTE_A5, 1);
        break;
    case LOG_ERR:
        sns_beep(fd, SNS_BEEP_NOTE_A4, .5);
        break;
    case LOG_WARNING:
        sns_beep(fd, SNS_BEEP_NOTE_A3, .5);
        break;
    default: ;
    }
}


/* static void update(cx_t *cx) { */
/*     int r; */
/*     struct timespec abstimeout = aa_tm_future( aa_tm_sec2timespec(1) ); */

/*     // get message */
/*     Somatic__Event *msg = SOMATIC_D_GET( &r, somatic__event, &cx->d, */
/*                                          &cx->chan, &abstimeout, ACH_O_WAIT ); */
/*     // validate */
/*     // log */
/*     if( msg ) { */
/*         aa_mem_region_t *reg = &cx->d.memreg; */
/*         char *head = aa_mem_region_printf( reg, */
/*                                        "[%s].(%s)", */
/*                                        msg->ident ? msg->ident : "", */
/*                                        msg->has_code ? somatic_event_code2str(msg->code) : "?" ); */
/*         const char *type = msg->type ? aa_mem_region_printf(reg, ".(%s)", msg->type) : ""; */
/*         const char *proc = */
/*             ( msg->has_code && */
/*               (SOMATIC__EVENT__CODES__PROC_STARTING == msg->code || */
/*                SOMATIC__EVENT__CODES__PROC_RUNNING == msg->code || */
/*                SOMATIC__EVENT__CODES__PROC_STOPPING == msg->code || */
/*                SOMATIC__EVENT__CODES__PROC_HALTED == msg->code ) ) ? */
/*               aa_mem_region_printf(reg, " %d@%s", */
/*                                msg->has_pid ? msg->pid : 0, */
/*                                msg->host ? msg->host : "?") : ""; */
/*         const char *comment = */
/*             msg->comment ? aa_mem_region_printf(reg, " %s", msg->comment) : ""; */
/*         int pri; */
/*         if( msg->has_priority  && msg->priority <= LOG_DEBUG */
/*             /\* unsigned, always true && msg->priority >= LOG_EMERG*\/ ) { */
/*             pri = msg->priority; */
/*         } else { pri = LOG_ERR; } */
/*         syslog(pri, "%s%s%s%s", head, type, proc, comment ); */
/*     } */
/*     // beep */
/*     if( msg && msg->has_priority) { */
/*         switch(msg->priority) { */
/*         case SOMATIC__EVENT__PRIORITIES__EMERG: */
/*             somatic_beep(cx->beepfd, 3500, 3); */
/*             break; */
/*         case SOMATIC__EVENT__PRIORITIES__ALERT: */
/*             somatic_beep(cx->beepfd, 2500, 2); */
/*             break; */
/*         case SOMATIC__EVENT__PRIORITIES__CRIT: */
/*             somatic_beep(cx->beepfd, 1500, 1); */
/*             break; */
/*         default: ; */
/*         } */
/*     } */
/* } */
