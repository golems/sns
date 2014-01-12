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


#include "sns.h"


#define SNS_MSG_PLUGIN_DUMP(type) type ## _dump
#define SNS_MSG_PLUGIN_PLOT_SAMPLE(type) type ## _plot_sample

/* Define the plugin entry points, dispatching to the appropriate
 * functions */
#define SNS_MSG_PLUGIN_DEFINE(type)                                     \
    void sns_msg_dump                                                   \
    ( FILE *out, const void *msg )                                      \
    {                                                                   \
        SNS_MSG_PLUGIN_DUMP(type)( out, (struct type*) msg );           \
    }                                                                   \
                                                                        \
    void sns_msg_plot_sample                                            \
    ( const void *msg,                                                  \
      double **sample_ptr, char ***sample_labels, size_t *sample_size ) \
    {                                                                   \
        SNS_MSG_PLUGIN_PLOT_SAMPLE(type)                                \
            ( (struct type*)msg,                                        \
              sample_ptr, sample_labels, sample_size );                 \
    }                                                                   \

/* Define SNS_MSG_PLUGIN_TYPE on the command line */
SNS_MSG_PLUGIN_DEFINE( SNS_MSG_PLUGIN_TYPE );
