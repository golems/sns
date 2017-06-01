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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif /* HAVE_CONFIG_H */

#include "sns.h"
#include <ach/experimental.h>
#include "sns/event.h"


static enum ach_status
sns_evhandle_impl( struct sns_evhandler *cx, ach_channel_t *channel, const struct timespec *timeout, int ach_options )
{
    /* get message */
    void *buf = NULL;
    size_t frame_size;
    enum ach_status r = sns_msg_local_get( channel, &buf,
                                           &frame_size,
                                           timeout, ach_options );

    /* maybe do something */
    if( ach_status_match(r, ACH_MASK_OK | ACH_MASK_MISSED_FRAME) ) {
        assert(buf);
        r = cx->handler( cx->context, buf, frame_size );
        aa_mem_region_local_pop(buf);
    } else {
        assert( NULL == buf );
    }

    return r;
}


static enum ach_status
sns_evhandle_fun( void *_cx, ach_channel_t *channel )
{
    struct sns_evhandler *cx = (struct sns_evhandler *) _cx;
    return sns_evhandle_impl(cx, channel, NULL, cx->ach_options);
}

static void check_evhandle_result( enum ach_status r )
{
    SNS_REQUIRE( ach_status_match(r, ACH_MASK_OK | ACH_MASK_CANCELED | ACH_MASK_TIMEOUT),
                 "asdf Could not handle events: %s, %s\n",
                 ach_result_to_string(r),
                 strerror(errno) );
}
enum ach_status ACH_WARN_UNUSED
sns_evhandle( struct sns_evhandler *handlers,
              size_t n,
              const struct timespec *period,
              enum ach_status (*periodic_handler)(void *context),
              void *periodic_context,
              int *cancel_sigs,
              int options )
{
    /* Create ach handlers */
    /* Install cancel handler */
    if( cancel_sigs ) {
        ach_channel_t *chans[n+1];
        for( size_t i = 0; i < n; i ++ ) {
            chans[i] = handlers[i].channel;
        }
        chans[n] = NULL;
        sns_sigcancel(chans, cancel_sigs);
    }

    if( n == 1 ) {
        /* special case single channel so we can handle userspace */
        while( !sns_cx.shutdown) {
            errno = 0;
            enum ach_status r = sns_evhandle_impl( handlers, handlers->channel,
                                                   period, handlers->ach_options | ACH_O_RELTIME | ACH_O_WAIT );
            if(sns_cx.shutdown) break;
            check_evhandle_result(r);
            if( periodic_handler ) {
                periodic_handler( periodic_context );
            }
        }
    } else {
        /* multiple channels, use ach event loop */
        struct ach_evhandler ach_handlers[n];
        for( size_t i = 0; i < n; i ++ ) {
            ach_handlers[i].context = handlers + i;
            ach_handlers[i].channel = handlers[i].channel;
            ach_handlers[i].handler = sns_evhandle_fun;
        }

        while( !sns_cx.shutdown) {
            errno = 0;
            enum ach_status r = ach_evhandle( ach_handlers, n,
                                              period, periodic_handler, periodic_context,
                                              options );
            if(sns_cx.shutdown) break;
            check_evhandle_result(r);

        }
    }

    return ACH_OK;
}
