/*
 * Copyright (c) 2015, Rice University.
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

#ifndef SNS_EVENT_H
#define SNS_EVENT_H


/**
 * @file  event.h
 * @brief Event loop for SNS daemons
 *
 * @author Neil T. Dantam
 */


#ifdef __cplusplus
extern "C" {
#endif

/**
 * Control structure for event handling loop
 */
struct sns_evhandler {
    /**
     * Channel to get messages from
     */
    struct ach_channel *channel;

    /**
     * Context argument for handler
     */
    void *context;

    /**
     * options for ach_get()
     */
    int ach_options;

    /**
     * Handler function.
     *
     * Called whenever there is new data in the channel.
     *
     * msg is deallocated after handler returns.
     *
     * Handler should return ACH_OK when new frames are read, or
     * ACH_STALE_FRAMES if no new frames are read.  Any other
     * return value will terminate the event loop.
     */
    enum ach_status (*handler)
    ( void *context, void *msg, size_t msg_size );
};

/**
 * Event loop for handling multiple channels.
 *
 * @param[in,out]              handlers array of handler descriptors
 *
 * @param[in] n                size of handlers array
 *
 * @param[in] period           timeout to wait between execution of periodic
 *                             function when requested in flags.
 *
 * @param[in] periodic_handler function to executre periodicly,
 *                             i.e., when timeout occurs or when
 *                             new messages are received.
 *
 * @param[in] periodic_context context argument to the periodic_handler
 *
 * @param[in] cancel_sigs      install signal handler to call ach_cancel()
 *                             on signals in given zero-terminated array
 *
 * @param[in] options          bit flags, may include
 *                             ACH_EV_O_PERIODIC_INPUT and
 *                             ACH_EV_O_PERIODIC_TIMEOUT
 */
enum ach_status ACH_WARN_UNUSED
sns_evhandle( struct sns_evhandler *handlers,
              size_t n,
              const struct timespec *period,
              enum ach_status (*periodic_handler)(void *context),
              void *periodic_context,
              int *cancel_sigs,
              int options );

#ifdef __cplusplus
}
#endif

#endif /*SNS_EVENT_H*/
