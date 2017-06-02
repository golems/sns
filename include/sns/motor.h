/*
 * Copyright (c) 2015-2017 Rice University
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products
 *       derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#ifndef SNS_MOTOR_H
#define SNS_MOTOR_H

#include "sns.h"

/**
 * Descriptor to remap motor indices.
 *
 * If receiving state or references from multiple channels, each
 * channel containing only a subset of the scenegraph's configuration,
 * it will be necessary to remap the data from the messages into the
 * arrays for the full scenegraph.  Similary, if sending
 * state/references out onto multiple channels, each with only a
 * subset of the full configuration, the output must also be remapped.
 */
struct sns_motor_map {

    /**
     * The scene graph for this descriptor
     */
    const struct aa_rx_sg *sg;

    /**
     * Number of indicies described by this descriptor.
     *
     * Less than or equal to the configuration count of the scenegraph.
     */
    size_t n;

    /**
     * Names of the configuration variables.
     *
     * Size n
     */
    char **name;

    /**
     * Config ids of the variables.
     *
     * Size n
     */
    aa_rx_config_id *id;
};

/**
 * Fill q_all with values from q_sub, remapped according to M.
 */
AA_API void
sns_motor_map_in( const struct sns_motor_map *M,
                  size_t n_sub, const double *q_sub,
                  double *q_all );


/**
 * Write state to channel, remapping if necessary.
 */
AA_API void
sns_motor_map_state_out( const struct aa_ct_state *state,
                         const struct sns_motor_map *M,
                         const struct timespec *now, int64_t dur_ns,
                         struct ach_channel *channel );

/**
 * Destroy the motor map
 */
void
sns_motor_map_destroy( struct sns_motor_map *m );

/**
 * Parse str as a comma separated list.
 */
struct sns_motor_map *
sns_motor_map_parse( const char *str );

/**
 * Fill the config ids in motor map.
 */
int
sns_motor_map_fill_id( const struct aa_rx_sg *sg, struct sns_motor_map *M );


struct sns_motor_vref {
    size_t n;
    enum sns_motor_mode *mode;
    double *x;
    int *priority;
};


#endif /* SNS_MOTOR_H */
