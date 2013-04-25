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
    uint64_t from_pid;
    uint64_t seq;
    uint8_t from_host[SNS_HOSTNAME_LEN];
} sns_msg_header_t;

_Bool sns_msg_is_expired( const struct sns_msg_header *msg, const struct timespec *now );

void sns_msg_set_time( struct sns_msg_header *msg, const struct timespec *now, uint64_t duration_ns );

/**********/
/* MOTORS */
/**********/

enum sns_motor_mode {
    SNS_MOTOR_MODE_VEL = 0
};

struct sns_msg_motor_ref {
    struct sns_msg_header header;
    enum sns_motor_mode mode;
    uint32_t n;
    sns_real_t u[1];
};

struct sns_msg_motor_state {
    struct sns_msg_header header;
    enum sns_motor_mode mode;
    uint32_t size;
    struct {
        sns_real_t pos;
        sns_real_t vel;
        sns_real_t cur;
    } X[1];
};

/************/
/* JOYSTICK */
/************/

struct sns_msg_joystick {
    struct sns_msg_header header;
    uint64_t buttons;
    uint32_t n;
    sns_real_t axis[1];
};
static inline size_t sns_msg_joystick_size ( const struct sns_msg_joystick *msg ) {
    return sizeof(*msg) - sizeof(msg->axis[0]) + sizeof(msg->axis[0])*msg->n;
}

/*************************/
/* CONVENIENCE FUNCTIONS */
/*************************/

typedef void sns_msg_dump_fun( FILE *, void* );

struct sns_msg_motor_ref *sns_msg_motor_ref_alloc ( uint32_t n );
void sns_msg_motor_ref_dump ( FILE*, struct sns_msg_motor_ref *msg );

struct sns_msg_joystick *sns_msg_joystick_alloc ( uint32_t n );
void sns_msg_joystick_dump ( FILE*, struct sns_msg_joystick *msg );

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
