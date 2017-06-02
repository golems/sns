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

/**
 * Returns true if struct timespec x is greater than struct timespec y
 */
#define SNS_TIME_GT(x, y) (x.tv_sec > y.tv_sec || \
        (x.tv_sec == y.tv_sec && x.tv_nsec > y.tv_nsec))

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file msg.h
 *
 * @brief Message definitions for SNS daemons
 *
 * This file declares message types and convenience functions.
 * Message are contiguous C structs.  Variable length date is
 * implented using a flexible array member as the last field in the
 * struct.  Each message contains a metadata header (which includes
 * the length of the flexible array).
 *
 * @author Neil T. Dantam
 */

/***************/
/* Basic Types */
/***************/

/**
 * Type to use for SE(3) frames
 */
typedef struct aa_tf_qv sns_tf;

/**
 * Type to use for transformation frames and velocities
 */
typedef struct aa_tf_qv_dx sns_tf_dx;

/**
 * Weighted SE(3) transform type
 */
typedef struct {
    /**
     * The transform
     */
    sns_tf tf;

    /**
     * A weight for the tranformation
     */
    double weight;
} sns_wt_tf;

/***********/
/* HEADERS */
/***********/

/**
 * Metadata header for SNS messages
 *
 * Each message has this header has its first member.
 */
typedef struct sns_msg_header {
    int64_t sec;              ///< Time message sent, seconds portion
    int64_t dur_nsec;         ///< Valid message duration, nanoseconds
    uint32_t nsec;            ///< Time message sent, nanoseconds portion
    uint32_t n;               ///< Element count for variable-sized messages
    int64_t from_pid;         ///< Sending PID
    uint64_t seq;             ///< Sequence number
    /**
     * Sending Host
     */
    char from_host[SNS_HOSTNAME_LEN];
    /**
     * Sending process name
     */
    char ident[SNS_IDENT_LEN];
} sns_msg_header_t;

/**
 * Check if the message has expired.
 *
 * Message is expired if the message timestamp plus message duration
 * is after current time.
 *
 * @param[in] msg the message to check
 * @param[in] now the current time
 *
 * @return true when message is expired.  False when message is not
 * expired.
 */
int sns_msg_is_expired( const struct sns_msg_header *msg, const struct timespec *now );

/**
 * Set the time in the message
 *
 * @param[out] msg           the message in which the time is set
 * @param[in]  now           the message timestamp
 * @param[in]  duration_ns   the message duration in nanoseconds
 */
void sns_msg_set_time( struct sns_msg_header *msg, const struct timespec *now, int64_t duration_ns );

/**
 * Extract the message timestamp as a timespec
 *
 * @param[in] msg An SNS message
 *
 * @return the timestamp of msg
 */
static inline struct timespec
sns_msg_get_time( struct sns_msg_header *msg )
{
    struct timespec ts;
    ts.tv_sec = msg->sec;
    ts.tv_nsec = msg->nsec;
    return ts;
}

/**
 * Set message header metadata
 *
 * @post the members of msg are initialized
 *
 * @param[out] msg An SNS message
 */
void sns_msg_header_fill ( struct sns_msg_header *msg );

/* True if frame_size is too small */

/**
 * Check if a message is fully received
 *
 * @param[in] type       the type of the message
 * @param[in] pointer    the message buffer
 * @param[in] frame_size the number of bytes received
 */
#define SNS_MSG_CHECK_SIZE( type, pointer, frame_size )         \
    ( (frame_size) < sns_msg_ ## type ## _size_n(0) ||          \
      (frame_size) < sns_msg_ ## type ## _size(pointer) )


/***********/
/* MACROS */
/**********/


/**
 * Read an ACH message into buffer from the thread-local memory region
 *
 * @post Allocates message buffer from the thread-local memory region
 * and reads a message from the channel.
 *
 * @param[in]  chan        the channel from which the message is read
 * @param[out] pbuf        pointer to the buffer pointer
 * @param[out] frame_size  size of the received message
 * @param[in]  abstime     timeout for ach_get()
 * @param[in]  options     options for ach_get()
 */
enum ach_status
sns_msg_local_get( ach_channel_t *chan, void **pbuf,
                   size_t *frame_size,
                   const struct timespec *ACH_RESTRICT abstime,
                   int options );


/**
 * read an ACH message into buffer from the given memory region
 *
 * @post Allocates message buffer from the given memory region
 * and reads a message from the channel.
 *
 * @param[in]     chan        the channel from which the message is read
 * @param[in,out] region      memory region from which to allocate the message buffer
 * @param[out]    pbuf        pointer to the buffer pointer
 * @param[out]    frame_size  size of the received message
 * @param[in]     abstime     timeout for ach_get()
 * @param[in]     options     options for ach_get()
 */
enum ach_status
sns_msg_region_get( ach_channel_t *chan, struct aa_mem_region *region,
                    void **pbuf,
                    size_t *frame_size,
                    const struct timespec *ACH_RESTRICT abstime,
                    int options );

/**
 * Define many functions for vararray messages
 *
 */
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
    /* check size */                                                    \
    /* Returns difference between mem_size and recorded size  */        \
    static inline ssize_t                                               \
    type ## _check_size                                                 \
    ( struct type *msg, size_t mem_size )                               \
    {                                                                   \
        if( mem_size < sizeof(msg->header) ) {                          \
            return (ssize_t)mem_size - (ssize_t)sizeof(msg->header);    \
        }                                                               \
        return (ssize_t)mem_size - (ssize_t)type ## _size(msg);         \
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
    /* Allocate message in heap */                                      \
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

/**
 * Declare plugin functions for message type
 */
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

/**
 * Message type for log messages.
 */
typedef struct sns_msg_log {
    /**
     * Message header.
     */
    struct sns_msg_header header;
    /**
     * Log priority
     */
    int priority;
    /**
     * Log message text.
     */
    char text[1];
} sns_msg_log_t;

/**
 * Declare message functions.
 */
SNS_DEF_MSG_VAR( sns_msg_log, text );

/**
 * Declare message plugin functions.
 */
SNS_DEC_MSG_PLUGINS( sns_msg_log );

/**********/
/* Vector */
/**********/

/**
 * Message type for a floating point vector.
 */
struct sns_msg_vector {
    /**
     * Message header
     */
    struct sns_msg_header header;
    /**
     * vector elements
     */
    sns_real_t x[1];
};

/**
 * Declare message functions.
 */
SNS_DEF_MSG_VAR( sns_msg_vector, x );
/**
 * Declare message plugin functions.
 */
SNS_DEC_MSG_PLUGINS( sns_msg_vector );


/**********/
/* Matrix */
/**********/
/**
 * Message type for a floating point matrix
 */
struct sns_msg_matrix {
    /**
     * Message header
     */
    struct sns_msg_header header;
    /**
     * matrix rows
     */
    uint64_t rows;
    /**
     * matrix columns
     */
    uint64_t cols;
    /**
     * matrix elements
     */
    sns_real_t x[1];
};

/**
 * Compute message size of matrix with given rows and columns.
 */
static inline size_t sns_msg_matrix_size_mn ( size_t rows, size_t cols ) {
    static const struct sns_msg_matrix *msg;
    return sizeof(*msg) - sizeof(msg->x[0]) + sizeof(msg->x[0])*rows*cols;
}

/**
 * Compute message size of the given matrix.
 */
static inline size_t sns_msg_matrix_size ( const struct sns_msg_matrix *msg ) {
    return sns_msg_matrix_size_mn(msg->rows,msg->cols);
}


/**************/
/* Transforms */
/**************/

/* TF */
/**
 * Message type for SE(3) transforms
 */
struct sns_msg_tf {
   /**
    * The message header
    */
    struct sns_msg_header header;
    /**
     * transform elements
     */
    sns_tf tf[1];
};

/**
 * Declare message functions.
 */
SNS_DEF_MSG_VAR( sns_msg_tf, tf );
/**
 * Declare message plugin functions.
 */
SNS_DEC_MSG_PLUGINS( sns_msg_tf );

/* Weighted TF */
/**
 * Message type for weighted transforms
 */
struct sns_msg_wt_tf {
    /**
     * The message header
     */
    struct sns_msg_header header;
    /**
     * weighted transform elements
     */
    sns_wt_tf wt_tf[1];
};

/**
 * Declare message functions.
 */
SNS_DEF_MSG_VAR( sns_msg_wt_tf, wt_tf );
/**
 * Declare message plugin functions.
 */
SNS_DEC_MSG_PLUGINS( sns_msg_wt_tf );

/* TF DX */
/**
 * Message type for SE(3) transforms and velocity
 */
struct sns_msg_tf_dx {
    /**
     * The message header
     */
    struct sns_msg_header header;
    /**
     * the transform and velocity elements
     */
    sns_tf_dx tf_dx[1];
};

/**
 * Declare message functions.
 */
SNS_DEF_MSG_VAR( sns_msg_tf_dx, tf_dx );
/**
 * Declare message plugin functions.
 */
SNS_DEC_MSG_PLUGINS( sns_msg_tf_dx );

/**********/
/* MOTORS */
/**********/

/**
 * Type of commands for sns_msg_motor_ref messages
 */
enum sns_motor_mode {
    SNS_MOTOR_MODE_HALT = 1,
    SNS_MOTOR_MODE_POS  = 2,
    SNS_MOTOR_MODE_VEL  = 3,
    SNS_MOTOR_MODE_TORQ = 4,
    SNS_MOTOR_MODE_CUR = 5,
    SNS_MOTOR_MODE_RESET = 6,

    SNS_MOTOR_MODE_POS_OFFSET = 16,
};

/**
 * Return string describing the motor mode.
 */
AA_API const char *
sns_motor_mode_str( enum sns_motor_mode mode );

/**
 * Message type for motor commands
 */
struct sns_msg_motor_ref {
    /**
     * The message header
     */
    struct sns_msg_header header;
    /**
     * The type of command
     */
    enum sns_motor_mode mode;
    /**
     * The commanded values
     */
    sns_real_t u[1];
};

/**
 * Declare message functions.
 */
SNS_DEF_MSG_VAR( sns_msg_motor_ref, u );
/**
 * Declare message plugin functions.
 */
SNS_DEC_MSG_PLUGINS( sns_msg_motor_ref );

/**
 * Message type for tagged motor commands
 */
struct sns_msg_tag_motor_ref {
    /**
     * The message header
     */
    struct sns_msg_header header;
    /**
     * The type of command
     */
    enum sns_motor_mode mode;
    /**
     * Array of values and priority
     */
    struct {
            /**
             * The commanded value
             */
            sns_real_t val;

            /**
             * The priority of this value
             */
            uint64_t priority;
    } u[1];
};

/**
 * Declare message functions.
 */
SNS_DEF_MSG_VAR( sns_msg_tag_motor_ref, u );
/**
 * Declare message plugin functions.
 */
SNS_DEC_MSG_PLUGINS( sns_msg_tag_motor_ref );

/**
 * Message type for motor state
 */
struct sns_msg_motor_state {
    /**
     * The message header
     */
    struct sns_msg_header header;
    /**
     * The current mode of the motor
     */
    enum sns_motor_mode mode;


    /**
     * Array of motor state
     */
    struct {
        /**
         * The motor position
         */
        sns_real_t pos;
        /**
         * The motor velocity
         */
        sns_real_t vel;
        //sns_real_t cur;
    } X[1];
};

/**
 * Declare message functions.
 */
SNS_DEF_MSG_VAR( sns_msg_motor_state, X );
/**
 * Declare message plugin functions.
 */
SNS_DEC_MSG_PLUGINS( sns_msg_motor_state );

// TODO: use increment/leading-dimension/offset to store optional values

static inline double *
sns_msg_motor_state_pos( struct sns_msg_motor_state *msg )
{
    return &msg->X[0].pos;
}

static inline double *
sns_msg_motor_state_vel( struct sns_msg_motor_state *msg )
{
    return &msg->X[0].vel;
}

static inline double *
sns_msg_motor_state_acc( struct sns_msg_motor_state *msg )
{
    (void)msg;
    return NULL;
}

static inline double *
sns_msg_motor_state_eff( struct sns_msg_motor_state *msg )
{
    (void)msg;
    return NULL;
}

static inline size_t
sns_msg_motor_state_incpos( struct sns_msg_motor_state *msg )
{
    (void)msg;
    return 2;
}

static inline size_t
sns_msg_motor_state_incvel( struct sns_msg_motor_state *msg )
{
    (void)msg;
    return 2;
}

static inline size_t
sns_msg_motor_state_incacc( struct sns_msg_motor_state *msg )
{
    (void)msg;
    return 0;
}

static inline size_t
sns_msg_motor_state_inceff( struct sns_msg_motor_state *msg )
{
    (void)msg;
    return 0;
}

static inline uint32_t
sns_msg_motor_state_count( struct sns_msg_motor_state *msg )
{
    return msg->header.n;
}



/************/
/* JOYSTICK */
/************/

/**
 * Message type for joysticks and gamepads
 */
struct sns_msg_joystick {
    /**
     * The message header
     */
    struct sns_msg_header header;
    /**
     * Bit mask of joystick button state
     */
    uint64_t buttons;
    /**
     * Array of joystick axis state
     */
    sns_real_t axis[1];
};

/**
 * Declare message functions.
 */
SNS_DEF_MSG_VAR( sns_msg_joystick, axis );
/**
 * Declare message plugin functions.
 */
SNS_DEC_MSG_PLUGINS( sns_msg_joystick );

/*************************/
/* CONVENIENCE FUNCTIONS */
/*************************/

/** Allocate a motor_ref message */
struct sns_msg_motor_ref *sns_msg_motor_ref_alloc ( uint64_t n ) AA_DEPRECATED;

/***********/
/* PLUGINS */
/***********/

/**
 * Plugin function to print message to a FILE handle
 */
typedef void sns_msg_dump_fun( FILE *, void* );

/**
 * Plugin function to generate a plot sample for a message
 */
typedef void sns_msg_plot_sample_fun( const void *, double **, char ***, size_t *);

// TODO: message validation

/**
 * Load symbol from message plugin
 */
void *sns_msg_plugin_symbol( const char *type, const char *symbol );

/**
 * Declaration for the plugin dump function
 */
void sns_msg_dump( FILE *out, const void *msg ) ;

/**
 * Declaration for the plugin plog_sample function
 */
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
