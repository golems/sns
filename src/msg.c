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
    for(;;) {
        size_t size = aa_mem_region_freesize(reg);
        r = ach_get( chan, aa_mem_region_ptr(reg), size, frame_size, abstime, options );
        if( ACH_OVERFLOW == r ) aa_mem_region_tmpalloc( reg, *frame_size );
        else break;
    }

    if( ach_status_match(r, ACH_MASK_OK | ACH_MASK_MISSED_FRAME) )
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
        SNS_REQUIRE( dl_lib, "Couldn't open plugin '%s': %s\n", buf, dlerror() );
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

int sns_msg_is_expired( const struct sns_msg_header *msg, const struct timespec *now_arg ) {
    // FIXME: does this work with negative values?
    struct timespec now;
    int r = ensure_time( &now, now_arg );
    if( r ) return 0;

    struct timespec msg_time = {.tv_sec = msg->sec,
                                .tv_nsec = msg->nsec};
    struct timespec then = sns_time_add_ns( msg_time, msg->dur_nsec );

    return (SNS_TIME_GT(now, then));
}

void sns_msg_set_time( struct sns_msg_header *msg, const struct timespec *arg, int64_t dur_nsec ) {
    struct timespec now;
    ensure_time( &now, arg );
    msg->sec = now.tv_sec;
    msg->nsec = (uint32_t)now.tv_nsec;
    msg->dur_nsec = dur_nsec;
}

void sns_msg_header_fill ( struct sns_msg_header *msg ) {
    // lazily init
    if( !sns_cx.is_initialized ) sns_init();

    msg->from_pid = sns_cx.pid;

    assert( sizeof(msg->from_host) >= SNS_HOSTNAME_LEN ) ;
    assert( sizeof(sns_cx.host) >= SNS_HOSTNAME_LEN ) ;
    assert( sizeof(char) == sizeof(uint8_t) );

    strncpy( msg->from_host, sns_cx.host, SNS_HOSTNAME_LEN );

    assert( sizeof(msg->ident) >= SNS_IDENT_LEN ) ;
    strncpy( msg->ident, sns_cx.ident, SNS_HOSTNAME_LEN );

}


static void dump_header( FILE *out, const struct sns_msg_header *msg, const char *type ) {
    int64_t
        h = msg->sec / (60*60),
        m = msg->sec / 60 - h*60,
        s = msg->sec % 60;

    /* time: Thour:min:sec.nsec */
    fprintf( out, "[%s] %09"PRIu64" T%03"PRId64":%02"PRId64":%02"PRId64".%09"PRIu32" + %08"PRId64"\n",
             type, msg->seq, h, m, s, msg->nsec, msg->dur_nsec );

}

/*---- vector ----*/
void sns_msg_vector_plot_sample(
    const struct sns_msg_vector *msg, double **sample_ptr, char ***sample_labels, size_t *sample_size )
{
    aa_mem_region_t *reg = aa_mem_region_local_get();

    if( sample_ptr ) {
        *sample_ptr = (double*)aa_mem_region_alloc( reg, sizeof((*sample_ptr)[0]) * msg->header.n );
        for( size_t i = 0; i < msg->header.n; i ++ )
            (*sample_ptr)[i] = msg->x[i];
    }

    if( sample_labels ) {
        *sample_labels = (char**)aa_mem_region_alloc( reg, sizeof((*sample_ptr)[0]) * msg->header.n );
        for( size_t i = 0; i < msg->header.n; i ++ )
        (*sample_labels)[i] = aa_mem_region_printf( reg, "%d", i );
    }

    if( sample_size )
        *sample_size = msg->header.n;
}
void sns_msg_vector_dump ( FILE *out, const struct sns_msg_vector *msg ) {
    dump_header( out, &msg->header, "vector" );
    for( uint32_t i = 0; i < msg->header.n; i ++ ) {
        fprintf(out, "\t%f", msg->x[i] );
    }
    fprintf( out, "\n" );
}

/*---- transform ----*/
void sns_msg_tf_dump ( FILE *out, const struct sns_msg_tf *msg ) {
    dump_header( out, &msg->header, "tf" );
    for( uint32_t i = 0; i < msg->header.n; i ++ ) {
        fprintf(out, "\t%d: [%f\t%f\t%f\t%f\t%f\t%f\t%f\t]\n",
                i,
                msg->tf[i].r.data[0],
                msg->tf[i].r.data[1],
                msg->tf[i].r.data[2],
                msg->tf[i].r.data[3],
                msg->tf[i].v.data[0],
                msg->tf[i].v.data[1],
                msg->tf[i].v.data[2] );
    }
    fprintf( out, "\n" );
}

void sns_msg_tf_plot_sample(
    const struct sns_msg_tf *msg, double **sample_ptr, char ***sample_labels, size_t *sample_size )
{
    aa_mem_region_t *reg = aa_mem_region_local_get();

    if( sample_ptr ) {
        *sample_ptr = (double*)aa_mem_region_alloc( reg, sizeof((*sample_ptr)[0]) * msg->header.n * 7 );
        for( size_t i = 0; i < msg->header.n; i ++ ) {
            (*sample_ptr)[i*7+0] = msg->tf[i].r.x;
            (*sample_ptr)[i*7+1] = msg->tf[i].r.y;
            (*sample_ptr)[i*7+2] = msg->tf[i].r.z;
            (*sample_ptr)[i*7+3] = msg->tf[i].r.w;
            (*sample_ptr)[i*7+4] = msg->tf[i].v.x;
            (*sample_ptr)[i*7+5] = msg->tf[i].v.y;
            (*sample_ptr)[i*7+6] = msg->tf[i].v.z;
        }
    }

    if( sample_labels ) {
        *sample_labels = (char**)aa_mem_region_alloc( reg, 7*msg->header.n*sizeof((*sample_labels)[0]) );
        for( size_t i = 0; i < msg->header.n; i ++ ) {
            (*sample_labels)[7*i+0] = aa_mem_region_printf( reg, "q_x %d", i );
            (*sample_labels)[7*i+1] = aa_mem_region_printf( reg, "q_y %d", i );
            (*sample_labels)[7*i+2] = aa_mem_region_printf( reg, "q_z %d", i );
            (*sample_labels)[7*i+3] = aa_mem_region_printf( reg, "q_w %d", i );
            (*sample_labels)[7*i+4] = aa_mem_region_printf( reg, "x %d", i );
            (*sample_labels)[7*i+5] = aa_mem_region_printf( reg, "y %d", i );
            (*sample_labels)[7*i+6] = aa_mem_region_printf( reg, "z %d", i );
        }
    }

    if( sample_size )
        *sample_size = 7*msg->header.n;
}

void sns_msg_wt_tf_dump ( FILE *out, const struct sns_msg_wt_tf *msg ) {
    dump_header( out, &msg->header, "wt_tf" );
    for( uint32_t i = 0; i < msg->header.n; i ++ ) {
        fprintf(out, "\t%d: (%f) [%f\t%f\t%f\t%f\t%f\t%f\t%f\t]\n",
                i,
                msg->wt_tf[i].weight,
                msg->wt_tf[i].tf.r.data[0],
                msg->wt_tf[i].tf.r.data[1],
                msg->wt_tf[i].tf.r.data[2],
                msg->wt_tf[i].tf.r.data[3],
                msg->wt_tf[i].tf.v.data[0],
                msg->wt_tf[i].tf.v.data[1],
                msg->wt_tf[i].tf.v.data[2] );
    }
    fprintf( out, "\n" );
}

void sns_msg_wt_tf_plot_sample(
    const struct sns_msg_wt_tf *msg, double **sample_ptr, char ***sample_labels, size_t *sample_size )
{
    aa_mem_region_t *reg = aa_mem_region_local_get();
    size_t size = msg->header.n*8;
    if( sample_ptr ) {
        *sample_ptr = (double*)aa_mem_region_alloc( reg, sizeof((*sample_ptr)[0]) * size );
        for( size_t i = 0; i < msg->header.n; i ++ ) {
            (*sample_ptr)[i*8+0] = msg->wt_tf[i].weight;
            (*sample_ptr)[i*8+1] = msg->wt_tf[i].tf.r.x;
            (*sample_ptr)[i*8+2] = msg->wt_tf[i].tf.r.y;
            (*sample_ptr)[i*8+3] = msg->wt_tf[i].tf.r.z;
            (*sample_ptr)[i*8+4] = msg->wt_tf[i].tf.r.w;
            (*sample_ptr)[i*8+5] = msg->wt_tf[i].tf.v.x;
            (*sample_ptr)[i*8+6] = msg->wt_tf[i].tf.v.y;
            (*sample_ptr)[i*8+7] = msg->wt_tf[i].tf.v.z;
        }
    }

    if( sample_labels ) {
        *sample_labels = (char**)aa_mem_region_alloc( reg, size*sizeof((*sample_labels)[0]) );
        for( size_t i = 0; i < msg->header.n; i ++ ) {
            (*sample_labels)[8*i+0] = aa_mem_region_printf( reg, "wt %d", i );
            (*sample_labels)[8*i+1] = aa_mem_region_printf( reg, "q_x %d", i );
            (*sample_labels)[8*i+2] = aa_mem_region_printf( reg, "q_y %d", i );
            (*sample_labels)[8*i+3] = aa_mem_region_printf( reg, "q_z %d", i );
            (*sample_labels)[8*i+4] = aa_mem_region_printf( reg, "q_w %d", i );
            (*sample_labels)[8*i+5] = aa_mem_region_printf( reg, "x %d", i );
            (*sample_labels)[8*i+6] = aa_mem_region_printf( reg, "y %d", i );
            (*sample_labels)[8*i+7] = aa_mem_region_printf( reg, "z %d", i );
        }
    }

    if( sample_size )
        *sample_size = size;
}



void sns_msg_tf_dx_dump ( FILE *out, const struct sns_msg_tf_dx *msg ) {
    dump_header( out, &msg->header, "tf_dx" );
    for( uint32_t i = 0; i < msg->header.n; i ++ ) {
        fprintf(out,
                "\t%d: [%f\t%f\t%f\t%f]\t[%f\t%f\t%f\t]\n"
                "\t    [%f\t%f\t%f\t|\t%f\t%f\t%f\t]\n",
                i,
                msg->tf_dx[i].tf.r.data[0],
                msg->tf_dx[i].tf.r.data[1],
                msg->tf_dx[i].tf.r.data[2],
                msg->tf_dx[i].tf.r.data[3],
                msg->tf_dx[i].tf.v.data[0],
                msg->tf_dx[i].tf.v.data[1],
                msg->tf_dx[i].tf.v.data[2],
                msg->tf_dx[i].dx.dv[0], msg->tf_dx[i].dx.dv[1], msg->tf_dx[i].dx.dv[2],
                msg->tf_dx[i].dx.omega[0], msg->tf_dx[i].dx.omega[1], msg->tf_dx[i].dx.omega[2]
            );
    }
    fprintf( out, "\n" );
}



/*---- motor_ref ----*/
const char *
sns_motor_mode_str( enum sns_motor_mode mode )
{
    switch( mode ) {
    case SNS_MOTOR_MODE_HALT:
        return "halt";
        break;
    case SNS_MOTOR_MODE_POS:
        return "position";
        break;
    case SNS_MOTOR_MODE_VEL:
        return "velocity";
        break;
    case SNS_MOTOR_MODE_TORQ:
        return "torque";
        break;
    case SNS_MOTOR_MODE_CUR:
        return "current";
        break;
    case SNS_MOTOR_MODE_POS_OFFSET:
        return "position offset";
        break;
    case SNS_MOTOR_MODE_RESET:
        return "reset";
        break;
    }
    return "?";
};

/*---- motor_ref ----*/
struct sns_msg_motor_ref *sns_msg_motor_ref_alloc ( uint64_t n ) {
    return sns_msg_motor_ref_heap_alloc( (uint32_t)n );
}

void sns_msg_motor_ref_dump ( FILE *out, const struct sns_msg_motor_ref *msg ) {
    dump_header( out, &msg->header, "motor_ref" );
    const char *mode = "?";
    switch( msg->mode ) {
    case SNS_MOTOR_MODE_HALT:       mode = "halt";             break;
    case SNS_MOTOR_MODE_POS:        mode = "position";         break;
    case SNS_MOTOR_MODE_VEL:        mode = "velocity";         break;
    case SNS_MOTOR_MODE_TORQ:       mode = "torque";           break;
    case SNS_MOTOR_MODE_CUR:        mode = "current";          break;
    case SNS_MOTOR_MODE_POS_OFFSET: mode = "position offset";  break;
    case SNS_MOTOR_MODE_RESET:      mode = "reset";  break;
    }
    fprintf(out, "\t%s\n", mode );
    for( uint32_t i = 0; i < msg->header.n; i ++ ) {
        fprintf(out, "\t%f", msg->u[i] );
    }
    fprintf( out, "\n" );
}

void sns_msg_motor_ref_plot_sample(
    const struct sns_msg_motor_ref *msg, double **sample_ptr, char ***sample_labels, size_t *sample_size )
{
    aa_mem_region_t *reg = aa_mem_region_local_get();

    if( sample_ptr ) {
        *sample_ptr = (double*)aa_mem_region_alloc( reg, sizeof((*sample_ptr)[0]) * msg->header.n );
        for( size_t i = 0; i < msg->header.n; i ++ )
            (*sample_ptr)[i] = msg->u[i];
    }

    if( sample_labels ) {
        *sample_labels = (char**)aa_mem_region_alloc( reg, sizeof((*sample_ptr)[0]) * msg->header.n );
        for( size_t i = 0; i < msg->header.n; i ++ )
        (*sample_labels)[i] = aa_mem_region_printf( reg, "%d", i );
    }

    if( sample_size )
        *sample_size = msg->header.n;
}

/*---- tag_motor_ref ----*/
void sns_msg_tag_motor_ref_dump ( FILE *out, const struct sns_msg_tag_motor_ref *msg ) {
    dump_header( out, &msg->header, "tag_motor_ref" );
    const char *mode = "?";
    switch( msg->mode ) {
    case SNS_MOTOR_MODE_HALT:       mode = "halt";             break;
    case SNS_MOTOR_MODE_POS:        mode = "position";         break;
    case SNS_MOTOR_MODE_VEL:        mode = "velocity";         break;
    case SNS_MOTOR_MODE_TORQ:       mode = "torque";           break;
    case SNS_MOTOR_MODE_CUR:        mode = "current";          break;
    case SNS_MOTOR_MODE_POS_OFFSET: mode = "position offset";  break;
    case SNS_MOTOR_MODE_RESET:      mode = "reset";  break;
    }
    fprintf(out, "\t%s\n", mode );
    for( uint32_t i = 0; i < msg->header.n; i ++ )
        fprintf(out, "\t(%f,%lu)", msg->u[i].val, msg->u[i].priority );

    fprintf( out, "\n" );
}

void sns_msg_tag_motor_ref_plot_sample(
    const struct sns_msg_tag_motor_ref *msg, double **sample_ptr, char ***sample_labels, size_t *sample_size )
{
    aa_mem_region_t *reg = aa_mem_region_local_get();

    if( sample_ptr ) {
        *sample_ptr = (double*)aa_mem_region_alloc( reg, sizeof((*sample_ptr)[0]) * msg->header.n );
        for( size_t i = 0; i < msg->header.n; i ++ )
            (*sample_ptr)[i] = msg->u[i].val;
    }

    if( sample_labels ) {
        *sample_labels = (char**)aa_mem_region_alloc( reg, sizeof((*sample_ptr)[0]) * msg->header.n );
        for( size_t i = 0; i < msg->header.n; i ++ )
        (*sample_labels)[i] = aa_mem_region_printf( reg, "%d", i );
    }

    if( sample_size )
        *sample_size = msg->header.n;
}

/*---- motor_state ----*/
struct sns_msg_motor_state *sns_msg_motor_state_alloc ( uint32_t n ) {
    return sns_msg_motor_state_heap_alloc(n);
}
void sns_msg_motor_state_dump ( FILE *out, const struct sns_msg_motor_state *msg ) {
    dump_header( out, &msg->header, "motor_state" );
    for( uint32_t i = 0; i < msg->header.n; i ++ ) {
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
        *sample_ptr = (double*)aa_mem_region_alloc( reg, sizeof((*sample_ptr)[0]) * msg->header.n * 2 );
        for( size_t i = 0, j=0; i < msg->header.n; i ++ ) {
            (*sample_ptr)[j++] = msg->X[i].pos;
            (*sample_ptr)[j++] = msg->X[i].vel;
        }
    }

    if( sample_labels ) {
        *sample_labels = (char**)aa_mem_region_alloc( reg, sizeof((*sample_ptr)[0]) * msg->header.n * 2 );
        for( size_t i = 0, j = 0; i < msg->header.n; i ++ ) {
            (*sample_labels)[j++] = aa_mem_region_printf( reg, "pos %d", i );
            (*sample_labels)[j++] = aa_mem_region_printf( reg, "vel %d", i );
        }
    }

    if( sample_size )
        *sample_size = 2*msg->header.n;
}


/*---- joystick ----*/

void sns_msg_joystick_dump ( FILE *out, const struct sns_msg_joystick *msg ) {
    dump_header( out, &msg->header, "joystick" );
    fprintf( out, "0x%08"PRIx64, msg->buttons );
    for( uint32_t i = 0; i < msg->header.n; i ++ ) {
        fprintf(out, "\t%f", msg->axis[i] );
    }
    fprintf( out, "\n" );
}

void sns_msg_joystick_plot_sample(
    const struct sns_msg_joystick *msg, double **sample_ptr, char ***sample_labels, size_t *sample_size )
{
    aa_mem_region_t *reg = aa_mem_region_local_get();

    if( sample_ptr ) {
        *sample_ptr = (double*)aa_mem_region_alloc( reg, sizeof((*sample_ptr)[0]) * msg->header.n );
        for( size_t i = 0; i < msg->header.n; i ++ )
            (*sample_ptr)[i] = msg->axis[i];
    }

    if( sample_labels ) {
        *sample_labels = (char**)aa_mem_region_alloc( reg, sizeof((*sample_ptr)[0]) * msg->header.n );
        for( size_t i = 0; i < msg->header.n; i ++ )
        (*sample_labels)[i] = aa_mem_region_printf( reg, "%d", i );
    }

    if( sample_size )
        *sample_size = msg->header.n;
}
