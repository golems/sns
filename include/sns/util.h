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

#ifndef SNS_UTIL_H
#define SNS_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

int sns_beep( int fd, double freq, double dur );


#define SNS_BEEP_NOTE_A3    220
#define SNS_BEEP_NOTE_A4    440
#define SNS_BEEP_NOTE_A4S   466.16
#define SNS_BEEP_NOTE_B4    493.88
#define SNS_BEEP_NOTE_C5    523.25
#define SNS_BEEP_NOTE_C5S   554.37

#define SNS_BEEP_NOTE_A5    880

#define SNS_BEEP_NOTE_A6    1760

#define SNS_BEEP_NOTE_A7    3520

static inline struct timespec sns_time_add_ns( struct timespec ts, int64_t ns ) {
    int64_t ns1 = ns + ts.tv_nsec;
    struct timespec r = { .tv_nsec = ns1 % 1000000000,
                          .tv_sec = ts.tv_sec + ns1 / 1000000000 };
    return r;
}

const char *sns_str_nullterm( const char *text, size_t n );

unsigned long sns_parse_uhex( const char *arg, uint64_t max );

double sns_parse_float( const char *arg );

#ifdef __cplusplus
}
#endif
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/* Local Variables:                          */
/* mode: c                                   */
/* c-basic-offset: 4                         */
/* indent-tabs-mode:  nil                    */
/* End:                                      */
#endif //SNS_UTIL_H
