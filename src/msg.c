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
#include <assert.h>
#include <time.h>
#include <ach.h>
#include <dlfcn.h>
#include <syslog.h>
#include "sns.h"

enum ach_status
sns_msg_local_get( ach_channel_t *chan, void **pbuf, size_t *frame_size,
                   const struct timespec *ACH_RESTRICT abstime,
                   int options ) {
    aa_mem_region_t *reg = aa_mem_region_local_get();
    ach_status_t r;
    do {
        size_t size = aa_mem_region_freesize(reg);
        r = ach_get( chan, aa_mem_region_ptr(reg), size, frame_size, abstime, options );
        if( ACH_OVERFLOW == r ) {
            aa_mem_region_tmpalloc( reg, *frame_size );
        }
    } while (ACH_OVERFLOW == r );

    if( ACH_OK == r || ACH_MISSED_FRAME == r )
        *pbuf = aa_mem_region_alloc( reg, *frame_size );
    return r;
}


void *sns_msg_plugin_symbol( const char *type, const char *symbol ) {
    void *dl_lib;
    {
        const char prefix[] = "libsns_msg_";
        const char suffix[] = ".so";
        char buf[ strlen(prefix) + strlen(suffix) + strlen(type) + 1 ];
        strcpy(buf,prefix);
        strcat(buf,type);
        strcat(buf,suffix);
        dl_lib = dlopen(buf, RTLD_NOW);
        SNS_REQUIRE( dl_lib, "Couldn't open plugin '%s'\n", buf );
    }

    /*-- Obtain thing -- */
    return dlsym( dl_lib, symbol );
}


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
    struct timespec now;
    int r = ensure_time( &now, now_arg );
    if( r ) return 0;

    struct timespec msg_time = {.tv_sec = msg->time.sec,
                                .tv_nsec = msg->time.nsec};
    struct timespec then = sns_time_add_ns( msg_time, msg->time.dur_nsec );

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

void sns_msg_header_fill ( struct sns_msg_header *msg ) {
    msg->from_pid = sns_cx.pid;
    const char *host = sns_str_nullterm( sns_cx.host, SNS_HOSTNAME_LEN );
    memcpy( msg->from_host, host, strlen(host) );
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

/*---- vector ----*/
void sns_msg_vector_plot_sample(
    const struct sns_msg_vector *msg, double **sample_ptr, char ***sample_labels, size_t *sample_size )
{
    aa_mem_region_t *reg = aa_mem_region_local_get();

    if( sample_ptr ) {
        *sample_ptr = (double*)aa_mem_region_alloc( reg, sizeof((*sample_ptr)[0]) * msg->n );
        for( size_t i = 0; i < msg->n; i ++ )
            (*sample_ptr)[i] = msg->x[i];
    }

    if( sample_labels ) {
        *sample_labels = (char**)aa_mem_region_alloc( reg, sizeof((*sample_ptr)[0]) * msg->n );
        for( size_t i = 0; i < msg->n; i ++ )
        (*sample_labels)[i] = aa_mem_region_printf( reg, "%d", i );
    }

    if( sample_size )
        *sample_size = msg->n;
}
void sns_msg_vector_dump ( FILE *out, const struct sns_msg_vector *msg ) {
    dump_header( out, &msg->header, "vector" );
    for( uint32_t i = 0; i < msg->n; i ++ ) {
        fprintf(out, "\t%f", msg->x[i] );
    }
    fprintf( out, "\n" );
}


/*---- motor_ref ----*/
struct sns_msg_motor_ref *sns_msg_motor_ref_alloc ( uint32_t n ) {
    size_t size = sns_msg_motor_ref_size( & (struct sns_msg_motor_ref){.n=n} );
    struct sns_msg_motor_ref *msg =
        (struct sns_msg_motor_ref*)malloc( size );
    memset(msg,0,sizeof(*msg));
    msg->n = n;
    return msg;
}

struct sns_msg_motor_ref *sns_msg_motor_ref_local_alloc ( uint32_t n ) {
    size_t size = sns_msg_motor_ref_size( & (struct sns_msg_motor_ref){.n=n} );
    struct sns_msg_motor_ref *msg =
        (struct sns_msg_motor_ref*)aa_mem_region_local_alloc( size );
    memset(msg,0,sizeof(*msg));
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

void sns_msg_motor_ref_plot_sample(
    const struct sns_msg_motor_ref *msg, double **sample_ptr, char ***sample_labels, size_t *sample_size )
{
    aa_mem_region_t *reg = aa_mem_region_local_get();

    if( sample_ptr ) {
        *sample_ptr = (double*)aa_mem_region_alloc( reg, sizeof((*sample_ptr)[0]) * msg->n );
        for( size_t i = 0; i < msg->n; i ++ )
            (*sample_ptr)[i] = msg->u[i];
    }

    if( sample_labels ) {
        *sample_labels = (char**)aa_mem_region_alloc( reg, sizeof((*sample_ptr)[0]) * msg->n );
        for( size_t i = 0; i < msg->n; i ++ )
        (*sample_labels)[i] = aa_mem_region_printf( reg, "%d", i );
    }

    if( sample_size )
        *sample_size = msg->n;
}


/*---- motor_state ----*/
struct sns_msg_motor_state *sns_msg_motor_state_alloc ( uint32_t n ) {
    size_t size = sns_msg_motor_state_size( & (struct sns_msg_motor_state){.n=n} );
    struct sns_msg_motor_state *msg =
        (struct sns_msg_motor_state*)malloc( size );
    memset(msg,0,sizeof(*msg));
    msg->n = n;
    return msg;
}
void sns_msg_motor_state_dump ( FILE *out, const struct sns_msg_motor_state *msg ) {
    dump_header( out, &msg->header, "motor_state" );
    for( uint32_t i = 0; i < msg->n; i ++ ) {
        fprintf(out, "\t(%f,%f) ",
                msg->X[i].pos, msg->X[i].vel );
    }
    fprintf(out,"\n");

}

void sns_msg_motor_state_plot_sample(
    const struct sns_msg_motor_state *msg, double **sample_ptr, char ***sample_labels, size_t *sample_size )
{
    aa_mem_region_t *reg = aa_mem_region_local_get();

    if( sample_ptr ) {
        *sample_ptr = (double*)aa_mem_region_alloc( reg, sizeof((*sample_ptr)[0]) * msg->n * 2 );
        for( size_t i = 0, j=0; i < msg->n; i ++ ) {
            (*sample_ptr)[j++] = msg->X[i].pos;
            (*sample_ptr)[j++] = msg->X[i].vel;
        }
    }

    if( sample_labels ) {
        *sample_labels = (char**)aa_mem_region_alloc( reg, sizeof((*sample_ptr)[0]) * msg->n * 2 );
        for( size_t i = 0, j = 0; i < msg->n; i ++ ) {
            (*sample_labels)[j++] = aa_mem_region_printf( reg, "pos %d", i );
            (*sample_labels)[j++] = aa_mem_region_printf( reg, "vel %d", i );
        }
    }

    if( sample_size )
        *sample_size = 2*msg->n;
}


/*---- joystick ----*/

struct sns_msg_joystick *sns_msg_joystick_alloc ( uint32_t n ) {
    size_t size = sns_msg_joystick_size( & (struct sns_msg_joystick){.n=n} );
    struct sns_msg_joystick *msg = (struct sns_msg_joystick*) malloc( size );
    memset(msg,0,sizeof(*msg));
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

void sns_msg_joystick_plot_sample(
    const struct sns_msg_joystick *msg, double **sample_ptr, char ***sample_labels, size_t *sample_size )
{
    aa_mem_region_t *reg = aa_mem_region_local_get();

    if( sample_ptr ) {
        *sample_ptr = (double*)aa_mem_region_alloc( reg, sizeof((*sample_ptr)[0]) * msg->n );
        for( size_t i = 0; i < msg->n; i ++ )
            (*sample_ptr)[i] = msg->axis[i];
    }

    if( sample_labels ) {
        *sample_labels = (char**)aa_mem_region_alloc( reg, sizeof((*sample_ptr)[0]) * msg->n );
        for( size_t i = 0; i < msg->n; i ++ )
        (*sample_labels)[i] = aa_mem_region_printf( reg, "%d", i );
    }

    if( sample_size )
        *sample_size = msg->n;
}
