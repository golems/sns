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


/***************/
/* Basic Types */
/***************/

typedef struct aa_tf_qv sns_tf;
typedef struct aa_tf_qv_dx sns_tf_dx;

/** Weighted transform type */
typedef struct {
    sns_tf tf;
    double weight;
} sns_wt_tf;

/***********/
/* HEADERS */
/***********/

/* Each message begins with a header */
typedef struct sns_msg_header {
    int64_t sec;              ///< Time message sent, seconds portion
    int64_t dur_nsec;         ///< Valid message duration, nanoseconds
    uint32_t nsec;            ///< Time message sent, nanoseconds portion
    uint32_t n;               ///< Element count for variable-sized messages
    int64_t from_pid;         ///< Sending PID
    uint64_t seq;             ///< Sequence number
    /// Sending Host
    char from_host[SNS_HOSTNAME_LEN];
    /// Sending process name
    char ident[SNS_IDENT_LEN];
} sns_msg_header_t;

int sns_msg_is_expired( const struct sns_msg_header *msg, const struct timespec *now );

void sns_msg_set_time( struct sns_msg_header *msg, const struct timespec *now, int64_t duration_ns );

static inline struct timespec
sns_msg_get_time( struct sns_msg_header *msg )
{
    struct timespec ts;
    ts.tv_sec = msg->sec;
    ts.tv_nsec = msg->nsec;
    return ts;
}

void sns_msg_header_fill ( struct sns_msg_header *msg );

/* Return 0 is frame_size is too small */
#define SNS_MSG_CHECK_SIZE( type, pointer, frame_size )         \
    ( (frame_size) < sns_msg_ ## type ## _size_n(0) ||          \
      (frame_size) < sns_msg_ ## type ## _size(pointer) )


/***********/
/* MACROS */
/**********/

// read an ACH message into buffer from the thread-local memory region
enum ach_status
sns_msg_local_get( ach_channel_t *chan, void **pbuf,
                   size_t *frame_size,
                   const struct timespec *ACH_RESTRICT abstime,
                   int options );

/* Define many functions for vararray messages */
#define SNS_DEF_MSG_VAR( type, var )                                    \
    /* size_n */                                                        \
    /* Returns size (in octets) necessary to hold n items  */           \
    static inline uint32_t                                              \
    type ## _size_n                                                     \
    ( uint32_t n )                                                      \
    {                                                                   \
        static const struct type *msg;                                  \
        return (uint32_t)( sizeof(*msg) -                               \
                         sizeof(msg->var[0]) +                          \
                         n*sizeof(msg->var[0]) );                       \
    }                                                                   \
    /* size */                                                          \
    /* Returns actual size (in octets) of msg, */                       \
    /* based on its count variable */                                   \
    static inline uint32_t                                              \
    type ## _size                                                       \
    ( struct type *msg )                                                \
    {                                                                   \
        return type ## _size_n( msg->header.n );;                       \
    }                                                                   \
    /* init */                                                          \
    /* Initialize a message */                                          \
    static inline void                                                  \
    type ## _init                                                       \
    ( struct type *msg, uint32_t n )                                    \
    {                                                                   \
        memset(msg, 0, type ## _size_n(n) );                            \
        sns_msg_header_fill( &msg->header );                            \
        msg->header.n = n;                                              \
    }                                                                   \
    /* alloc */                                                         \
    /* Allocate message in heat */                                      \
    static inline struct type*                                          \
    type ## _heap_alloc                                                 \
    ( uint32_t n )                                                      \
    {                                                                   \
        struct type *msg = (struct type *) malloc(type ## _size_n(n) ); \
        type ## _init(msg,n);                                           \
        return msg;                                                     \
    }                                                                   \
    /* region_alloc */                                                  \
    /* Allocate message from region */                                  \
    static inline struct type*                                          \
    type ## _region_alloc                                               \
    ( struct aa_mem_region *reg, uint32_t n )                           \
    {                                                                   \
        struct type *msg =                                              \
            (struct type *) aa_mem_region_alloc( reg,                   \
                                                 type ## _size_n(n) );  \
        type ## _init(msg,n);                                           \
        return msg;                                                     \
    }                                                                   \
    /* local_alloc */                                                   \
    /* Allocate message from thread-local region */                     \
    static inline struct type*                                          \
    type ## _local_alloc                                                \
    ( uint32_t n )                                                      \
    {                                                                   \
        return type ## _region_alloc( aa_mem_region_local_get(), n );   \
    }                                                                   \
    /* put */                                                           \
    /* Put message to channel */                                        \
    static inline enum ach_status                                       \
    type ## _put                                                        \
    ( ach_channel_t *chan, struct type *msg )                           \
    {                                                                   \
        return ach_put( chan, msg, type ## _size(msg) );                \
    }                                                                   \
    /* local_get */                                                     \
    /* Get message, allocated from local memory region */               \
    static inline enum ach_status                                       \
    type ## _local_get                                                  \
    ( ach_channel_t *chan, struct type **pmsg,                          \
      size_t *frame_size,                                               \
      const struct timespec *ACH_RESTRICT abstime,                      \
      int options )                                                     \
    {                                                                   \
        return  sns_msg_local_get ( chan,                               \
                                    (void**)pmsg, frame_size,           \
                                    abstime, options ) ;                \
    }


#define SNS_DEC_MSG_PLUGINS( type )                                     \
    void type ## _dump                                                  \
    ( FILE*, const struct type *msg );                                  \
    void type ## _plot_sample(                                          \
        const struct type *msg,                                         \
        double **sample_ptr,                                            \
        char ***sample_labels,                                          \
        size_t *sample_size );

/*******/
/* LOG */
/*******/

typedef struct sns_msg_log {
    struct sns_msg_header header;
    int priority;
    char text[1];
} sns_msg_log_t;

SNS_DEF_MSG_VAR( sns_msg_log, text );
SNS_DEC_MSG_PLUGINS( sns_msg_log );

/**********/
/* Vector */
/**********/

struct sns_msg_vector {
    struct sns_msg_header header;
    sns_real_t x[1];
};

SNS_DEF_MSG_VAR( sns_msg_vector, x );
SNS_DEC_MSG_PLUGINS( sns_msg_vector );


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


/**************/
/* Transforms */
/**************/

/* TF */
struct sns_msg_tf {
    struct sns_msg_header header;
    sns_tf tf[1];
};

SNS_DEF_MSG_VAR( sns_msg_tf, tf );
SNS_DEC_MSG_PLUGINS( sns_msg_tf );

/* Weighted TF */
struct sns_msg_wt_tf {
    struct sns_msg_header header;
    sns_wt_tf wt_tf[1];
};

SNS_DEF_MSG_VAR( sns_msg_wt_tf, wt_tf );
SNS_DEC_MSG_PLUGINS( sns_msg_wt_tf );

/* TF DX */
struct sns_msg_tf_dx {
    struct sns_msg_header header;
    sns_tf_dx tf_dx[1];
};

SNS_DEF_MSG_VAR( sns_msg_tf_dx, tf_dx );
SNS_DEC_MSG_PLUGINS( sns_msg_tf_dx );

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
    sns_real_t u[1];
};

SNS_DEF_MSG_VAR( sns_msg_motor_ref, u );
SNS_DEC_MSG_PLUGINS( sns_msg_motor_ref );

struct sns_msg_motor_state {
    struct sns_msg_header header;
    enum sns_motor_mode mode;
    struct {
        sns_real_t pos;
        sns_real_t vel;
        //sns_real_t cur;
    } X[1];
};

SNS_DEF_MSG_VAR( sns_msg_motor_state, X );
SNS_DEC_MSG_PLUGINS( sns_msg_motor_state );

/************/
/* JOYSTICK */
/************/

struct sns_msg_joystick {
    struct sns_msg_header header;
    uint64_t buttons;
    sns_real_t axis[1];
};

SNS_DEF_MSG_VAR( sns_msg_joystick, axis );
SNS_DEC_MSG_PLUGINS( sns_msg_joystick );

/*************************/
/* CONVENIENCE FUNCTIONS */
/*************************/



struct sns_msg_motor_ref *sns_msg_motor_ref_alloc ( uint64_t n );

/***********/
/* PLUGINS */
/***********/

typedef void sns_msg_dump_fun( FILE *, void* );
typedef void sns_msg_plot_sample_fun( const void *, double **, char ***, size_t *) ;

// TODO: message validation

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
