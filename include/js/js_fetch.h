/* -*- mode: C; c-basic-offset: 4  -*- */
/*
 * Copyright (c) 2009, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef JS_FETCH_H
#define JS_FETCH_H


static int 
js_fetch( double *axes, size_t n_axes, int32_t *buttons, 
          ach_channel_t *chan, int wait ) {
    // make buffers
    js_msg_t jsmsg;
    js_msg.axes = axes;
    js_msg.axes_max = n_axes;
    js_msg.axes_fill = n_axes;
    size_t n_js = js_frame_size( &jsmsg );
    size_t js_frame_size;
    uint8_t buf[n_js];
    // get message
    int r_ach = wait ?
        ach_wait_last( chan, buf, n_js, &js_frame_size, NULL ) :
        ach_get_last ( chan, buf, n_js, &js_frame_size );

    if( ACH_STALE_FRAMES == r_ach ) return 0;
    if( ACH_OK != r_ach ) return -1;
    
    // decode
    assert( ACH_OK == r_ach );
    int r_gm = js_msg_decode(&jsmsg, jsbuf, js_frame_size);
    if( r_gm < 0 ) return -1;

    // return
    *buttons = jsmsg.buttons;
    assert( axes[0] == jsmsg.axes[0] );

    return jsmsg.axes_fill;
}

#endif
