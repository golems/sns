/**
 * @file path.h
 * @brief Define struct to send path message
 * @date 2013/10/02 - last modified 2013/10/03
 */
#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sns.h>
#include <sns/msg.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * An enum for the possible results of blending a path.
 */
enum blend_status {
    OKAY, /* Blend completed okay. */
    DIVERGED, /* Blend did not complete; actual configuration diverged too much from the ideal. */
    // TODO: Add all control_msgs results.
};

/**
 * The reply message sent to the path sender when the path is finished executing.
 */
struct msg_path_result {
    struct sns_msg_header header; /* the message header, contains time sent. */
    enum blend_status status; /* the returned status. */
};


/**
 * @struct sns_msg_path_dense
 * @brief Struct that stores a densely sampled path
 * @todo
 */
struct sns_msg_path_dense {
    /**
     * The message header
     */
  struct sns_msg_header header;
  uint32_t n_dof;   /**< Degrees of Freedom (e.g. 7 for LWA) */
  uint32_t n_steps; /**< Number of points in the path */
  sns_real_t t0;        /**< Initial time */
  sns_real_t period;    /**< Message period (interval between points) */
  sns_real_t x[1];  /**< count is n_dof*n_steps */
};

/**
 * @function sns_msg_path_dense_size_tn
 * @brief Calculates the size of any message with its n_steps and n_dof
 */
static inline size_t sns_msg_path_dense_size_tn( size_t _n_steps,
                                               size_t _n_dofs ) {
  static const struct sns_msg_path_dense *msg;
  return sizeof( *msg ) - sizeof( msg->x[0] ) + sizeof( msg->x[0] )*_n_steps*_n_dofs;
}

/**
 * @function sns_msg_path_dense_size
 * @brief Returns the size of the message according to its n_steps and n_dof
 */
static inline size_t sns_msg_path_dense_size( const struct sns_msg_path_dense * _msg ) {
  return sns_msg_path_dense_size_tn( _msg->n_steps, _msg->n_dof );
}


  // DECLARATIONS
/**
 * Allocate message
 */
struct sns_msg_path_dense *sns_msg_path_dense_alloc( uint32_t _n_steps,
                                                     uint32_t _n_dof );
/**
 * print message
 */
void sns_path_dense_dump( FILE* _out,
                          const struct sns_msg_path_dense *_msg );


#ifdef __cplusplus
}
#endif
