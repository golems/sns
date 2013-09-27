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

#ifndef SNS_MSG_H
#define SNS_MSG_H

#ifdef __cplusplus
extern "C" {
#endif

/***********/
/* HEADERS */
/***********/

typedef struct sns_msg_time {
    int64_t sec;
    int64_t dur_nsec;
    uint32_t nsec;
} sns_msg_time_t;

// express duration in nanoseconds

typedef struct sns_msg_header {
    sns_msg_time_t time;
    int64_t from_pid;
    uint64_t seq;
    uint8_t from_host[SNS_HOSTNAME_LEN];
} sns_msg_header_t;

_Bool sns_msg_is_expired( const struct sns_msg_header *msg, const struct timespec *now );

 void sns_msg_set_time( struct sns_msg_header *msg, const struct timespec *now, int64_t duration_ns );


void sns_msg_header_fill ( struct sns_msg_header *msg );

/* Return 0 is frame_size is too small */
#define SNS_MSG_CHECK_SIZE( type, pointer, frame_size )         \
    ( (frame_size) < sns_msg_ ## type ## _size_n(0) ||          \
      (frame_size) < sns_msg_ ## type ## _size(pointer) )

 /*******/
 /* LOG */
 /******/

typedef struct sns_msg_log {
    struct sns_msg_header header;
    int priority;
    size_t n;
    char text[1];
} sns_msg_log_t;

static inline size_t sns_msg_log_size_n ( size_t n ) {
     static const struct sns_msg_log *msg;
     return sizeof(*msg) - sizeof(msg->text[0]) + sizeof(msg->text[0])*n;
 }

 static inline size_t sns_msg_log_size ( const struct sns_msg_log *msg ) {
     return sns_msg_log_size_n(msg->n);
 }

/**********/
/* Vector */
/**********/

struct sns_msg_vector {
    struct sns_msg_header header;
    uint64_t n;
    sns_real_t x[1];
};

static inline size_t sns_msg_vector_size_n ( size_t n ) {
    static const struct sns_msg_vector *msg;
    return sizeof(*msg) - sizeof(msg->x[0]) + sizeof(msg->x[0])*n;
}
static inline size_t sns_msg_vector_size ( const struct sns_msg_vector *msg ) {
    return sns_msg_vector_size_n(msg->n);
}
void sns_msg_vector_dump ( FILE*, const struct sns_msg_vector *msg );
void sns_msg_vector_plot_sample(
    const struct sns_msg_vector *msg, double **sample_ptr, char ***sample_labels, size_t *sample_size );


/**********/
/* Matrix */
/**********/

struct sns_msg_matrix {
    struct sns_msg_header header;
    uint64_t rows;
    uint64_t cols;
    sns_real_t x[1];
};

static inline size_t sns_msg_matrix_size_mn ( size_t rows, size_t cols ) {
    static const struct sns_msg_matrix *msg;
    return sizeof(*msg) - sizeof(msg->x[0]) + sizeof(msg->x[0])*rows*cols;
}
static inline size_t sns_msg_matrix_size ( const struct sns_msg_matrix *msg ) {
    return sns_msg_matrix_size_mn(msg->rows,msg->cols);
}

 /**********/
 /* MOTORS */
 /**********/

 enum sns_motor_mode {
     SNS_MOTOR_MODE_HALT = 1,
     SNS_MOTOR_MODE_POS  = 2,
     SNS_MOTOR_MODE_VEL  = 3,
     SNS_MOTOR_MODE_TORQ = 4
 };

 struct sns_msg_motor_ref {
     struct sns_msg_header header;
     enum sns_motor_mode mode;
     uint32_t n;
     sns_real_t u[1];
 };

 static inline size_t sns_msg_motor_ref_size_n ( size_t n ) {
     static const struct sns_msg_motor_ref *msg;
     return sizeof(*msg) - sizeof(msg->u[0]) + sizeof(msg->u[0])*n;
 }
 static inline size_t sns_msg_motor_ref_size ( const struct sns_msg_motor_ref *msg ) {
     return sns_msg_motor_ref_size_n(msg->n);
 }

 struct sns_msg_motor_state {
     struct sns_msg_header header;
     enum sns_motor_mode mode;
     uint32_t n;
     struct {
         sns_real_t pos;
         sns_real_t vel;
         //sns_real_t cur;
     } X[1];
 };

 static inline size_t sns_msg_motor_state_size_n ( size_t n ) {
     static const struct sns_msg_motor_state *msg;
     return sizeof(*msg) - sizeof(msg->X[0]) + sizeof(msg->X[0])*n;
 }
 static inline size_t sns_msg_motor_state_size ( const struct sns_msg_motor_state *msg ) {
     return sns_msg_motor_state_size_n(msg->n);
 }

 /************/
 /* JOYSTICK */
 /************/

 struct sns_msg_joystick {
     struct sns_msg_header header;
     uint64_t buttons;
     uint32_t n;
     sns_real_t axis[1];
 };

 static inline size_t sns_msg_joystick_size_n ( size_t n ) {
     static const struct sns_msg_joystick *msg;
     return sizeof(*msg) - sizeof(msg->axis[0]) + sizeof(msg->axis[0])*n;
 }
 static inline size_t sns_msg_joystick_size ( const struct sns_msg_joystick *msg ) {
     return sns_msg_joystick_size_n(msg->n);
 }


 /*************************/
 /* CONVENIENCE FUNCTIONS */
 /*************************/

 // read an ACH message into buffer from the thread-local memory region
 enum ach_status
 sns_msg_local_get( ach_channel_t *chan, void **pbuf,
                    size_t *frame_size,
                    const struct timespec *ACH_RESTRICT abstime,
                    int options );


struct sns_msg_motor_ref *sns_msg_motor_ref_alloc ( uint32_t n );

/** Allocate motor message of of local region */
struct sns_msg_motor_ref *sns_msg_motor_ref_local_alloc ( uint32_t n );

void sns_msg_motor_ref_dump ( FILE*, const struct sns_msg_motor_ref *msg );
void sns_msg_motor_ref_plot_sample(
    const struct sns_msg_motor_ref *msg, double **sample_ptr, char ***sample_labels, size_t *sample_size );

struct sns_msg_motor_state *sns_msg_motor_state_alloc ( uint32_t n );
void sns_msg_motor_state_dump ( FILE*, const struct sns_msg_motor_state *msg );
void sns_msg_motor_state_plot_sample(
    const struct sns_msg_motor_state *msg, double **sample_ptr, char ***sample_labels, size_t *sample_size );

struct sns_msg_joystick *sns_msg_joystick_alloc ( uint32_t n );
void sns_msg_joystick_dump ( FILE*, const struct sns_msg_joystick *msg );
void sns_msg_joystick_plot_sample(
    const struct sns_msg_joystick *msg, double **sample_ptr, char ***sample_labels, size_t *sample_size );


/***********/
/* PLUGINS */
/***********/

typedef void sns_msg_dump_fun( FILE *, void* );
typedef void sns_msg_plot_sample_fun( const void *, double **, char ***, size_t *) ;

void *sns_msg_plugin_symbol( const char *type, const char *symbol );
void sns_msg_dump( FILE *out, const void *msg ) ;
void sns_msg_plot_sample( const void *msg, double **sample_ptr, char ***sample_labels, size_t *sample_size ) ;

#ifdef __cplusplus
}
#endif
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/* Local Variables:                          */
/* mode: c                                   */
/* c-basic-offset: 4                         */
/* indent-tabs-mode:  nil                    */
/* End:                                      */
#endif //SNS_MSG_H
