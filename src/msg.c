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

// motor ref
struct sns_msg_motor_ref *sns_msg_motor_ref_alloc ( size_t n ) {
    static const struct sns_msg_motor_ref *msg = NULL;
    return (struct sns_msg_motor_ref*) malloc( sizeof(*msg) + n*sizeof(msg->u[0]) );
}

void sns_msg_motor_ref_dump ( struct sns_msg_motor_ref *msg ) {
    printf("msg:");
    for( uint32_t i = 0; i < msg->n; i ++ ) {
        fprintf(stdout, "\t%f", msg->u[i] );
    }
    printf("\n");
}

// joystick
struct sns_msg_motor_ref *sns_msg_joystick_alloc ( size_t n ) {
    static const struct sns_msg_joystick *msg = NULL;
    return (struct sns_msg_motor_ref*) malloc( sizeof(*msg) + n*sizeof(msg->axis[0]) );
}

void sns_msg_joystick_dump ( struct sns_msg_joystick *msg ) {
    printf("msg: %08"PRIx64, msg->buttons);
    for( uint32_t i = 0; i < msg->n; i ++ ) {
        fprintf(stdout, "\t%f", msg->axis[i] );
    }
    printf("\n");
}
