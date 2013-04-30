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


#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "sns.h"

/*---- Headers ----*/

static int ensure_time( struct timespec *now, const struct timespec *arg ) {
    if ( arg ) {
        memcpy( now, arg, sizeof(*now) );
        return 0;
    } else {
        return clock_gettime( ACH_DEFAULT_CLOCK, now );
    }
}

_Bool sns_msg_is_expired( const struct sns_msg_header *msg, const struct timespec *now_arg ) {
    // FIXME: does this work with negative values?
    struct timespec now, then;
    int r = ensure_time( &now, now_arg );
    if( r ) return 0;

    then.tv_sec = msg->time.sec + (msg->time.dur_nsec/1000000000);
    then.tv_nsec = msg->time.nsec + (msg->time.dur_nsec % 1000000000);

    return ( now.tv_sec > then.tv_sec ||
             (now.tv_sec == then.tv_sec &&
              now.tv_nsec > then.tv_nsec) );
}

void sns_msg_set_time( struct sns_msg_header *msg, const struct timespec *arg, int64_t dur_nsec ) {
    struct timespec now;
    ensure_time( &now, arg );
    msg->time.sec = now.tv_sec;
    msg->time.nsec = (uint32_t)now.tv_nsec;
    msg->time.dur_nsec = dur_nsec;
}


static void dump_header( FILE *out, const struct sns_msg_header *msg, const char *type ) {
    int64_t
        h = msg->time.sec / (60*60),
        m = msg->time.sec / 60 - h*60,
        s = msg->time.sec % 60;

    /* time: Thour:min:sec.nsec */
    fprintf( out, "[%s] %09"PRIu64" T%03"PRId64":%02"PRId64":%02"PRId64".%09"PRIu32" + %08"PRId64"\n",
             type, msg->seq, h, m, s, msg->time.nsec, msg->time.dur_nsec );

}

/*---- motor_ref ----*/

struct sns_msg_motor_ref *sns_msg_motor_ref_alloc ( uint32_t n ) {
    struct sns_msg_motor_ref *msg =
        (struct sns_msg_motor_ref*)malloc( sizeof(*msg) + n*sizeof(msg->u[0]) );
    msg->n = n;
    return msg;
}

void sns_msg_motor_ref_dump ( FILE *out, const struct sns_msg_motor_ref *msg ) {
    dump_header( out, &msg->header, "motor_ref" );
    for( uint32_t i = 0; i < msg->n; i ++ ) {
        fprintf(out, "\t%f", msg->u[i] );
    }
    fprintf( out, "\n" );
}

/*---- joystick ----*/

struct sns_msg_joystick *sns_msg_joystick_alloc ( uint32_t n ) {
    size_t size = sns_msg_joystick_size( & (struct sns_msg_joystick){.n=n} );

    struct sns_msg_joystick *msg = (struct sns_msg_joystick*) malloc( size );
    msg->n = n;
    return msg;
}

void sns_msg_joystick_dump ( FILE *out, const struct sns_msg_joystick *msg ) {
    dump_header( out, &msg->header, "joystick" );
    fprintf( out, "0x%08"PRIx64, msg->buttons );
    for( uint32_t i = 0; i < msg->n; i ++ ) {
        fprintf(out, "\t%f", msg->axis[i] );
    }
    fprintf( out, "\n" );
}
