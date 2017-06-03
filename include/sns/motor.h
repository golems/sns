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

/**
 * @file motor.h
 * @brief Convenience functions for motor message multiplexing.
 * @author Neil T. Dantam
 */

#include "sns.h"

/**
 * Forward declaration
 */
struct sns_evhandler;

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
 * Return the scenegraph for the map
*/
AA_API const struct aa_rx_sg *
sns_motor_map_sg(struct sns_motor_map *map);


/**
 * Descriptor for a motor message channel with axis remapping.
 */
struct sns_motor_channel {

    /** The actual channel */
    struct ach_channel channel;

    /** Channel name */
    const char *name;

    /** Remapping parameter */
    struct sns_motor_map *map;

    /** Priority for references from channel */
    int priority;

    /** Context for a handler */
    void *cx;

    /** Next element in list */
    struct sns_motor_channel *next;
};


/**
 * Motor reference metadata;
 */
struct sns_motor_ref_meta {

    /** Control mode */
    enum sns_motor_mode mode;

    /** Message time */
    struct timespec time;

    /** Expiration time */
    struct timespec expiration;

    /** Message priority */
    int priority;
};

/**
 * Motor references from a single message/channel.
 */
struct sns_motor_ref_elt {

    /* Channel to read messages from */
    struct sns_motor_channel *channel;

    /** Number of axes. */
    size_t n;

    /** Reference metadata */
    struct sns_motor_ref_meta meta;

    /** Value for each axis. */
    double *u;

};

/**
 * Return number of configurations described by the reference element.
 */
AA_API size_t
sns_motor_ref_elt_config_count( const struct sns_motor_ref_elt *e );

/**
 * Return remapping parameter (possibly NULL);
 */
AA_API struct sns_motor_map *
sns_motor_ref_elt_map( const struct sns_motor_ref_elt *e );

/**
 * A set of motor references.
 */
struct sns_motor_ref_set {

    /** Scenegraph for the reference set */
    const struct aa_rx_sg *scenegraph;

    /** Number of axes. */
    size_t n_q;

    /** Number of reference elements */
    size_t n_elt;

    /** Ref elements for channels */
    struct sns_motor_ref_elt *elt;

    /** Referance metadata */
    struct sns_motor_ref_meta *meta;

    /** Value for each axis. */
    double *u;
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

/**
 * Fill motor reference element from a single message
 */
AA_API void
sns_motor_ref_fill ( const struct sns_msg_motor_ref *msg,
                     struct sns_motor_ref_elt *ref_elt );

/**
 * Combine many reference messages into a single set.
 */
AA_API void
sns_motor_ref_collate ( const struct timespec *now,
                        struct sns_motor_ref_set *set );

/**
 * Add a new channel to the list.
 *
 * Does not open the channel.
 *
 * @sa sns_motor_channel_init()
 */
AA_API void
sns_motor_channel_push( const char *name, struct sns_motor_channel **plist );

/**
 * Post state to the motor channel, remapping as necessary.
 */
AA_API void
sns_motor_channel_put( struct sns_motor_channel *mc, const struct aa_ct_state *state,
                       const struct timespec *now, int64_t dur_ns );

/**
 * Parse axes in str as remap parameters for the motor channel.
 *
 * @param mc The motor channel
 * @param str Comma separated list of configuration variables to pass over the channel
 */
AA_API void
sns_motor_channel_parse_map( struct sns_motor_channel *mc, const char *str );

/**
 * Initialize variables in the motor channel list
 *
 * @pre scenegraph is initialized
 */

AA_API void
sns_motor_channel_init( struct sns_motor_channel *list, const struct aa_rx_sg *scenegraph );


/**
 * Initialize reference elements and reference set.
 */
AA_API void
sns_motor_ref_init( const struct aa_rx_sg *scenegraph,
                    struct sns_motor_channel *list,
                    struct sns_motor_ref_set **ref_set,
                    size_t n_handlers, struct sns_evhandler *handlers );

/**
 * Return the number of channel elements in list
 */
size_t
sns_motor_channel_count( struct sns_motor_channel *list );

#endif /* SNS_MOTOR_H */
