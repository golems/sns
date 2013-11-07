/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
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
/** Author: Neil Dantam
 */

#include "sns.h"
#include <linux/kd.h>
#include <sys/ioctl.h>
#include <unistd.h>


int sns_beep( int fd, double freq, double dur ) {
    // PC mainboard timer 8254 is clocked at 1.19 MHz
    static const double TICK_RATE =  1193180;
    static const double COUNT_RATE = 1000; // seems to work
    int tone = (int)(TICK_RATE / freq);
    int durticks = (int)(dur * COUNT_RATE);
    int argument = tone | (durticks << 16);
    if( ioctl(fd, KDMKTONE, argument) ) {
        perror("ioctl");
    }
    return 0;//return write(fd, "\a", 1);
}

const char *sns_str_nullterm( const char *text, size_t n ) {
    if( 0 == n ) return "";
    size_t i = strnlen(text, n);
    if( n == i ) {
        char *copy = (char *)aa_mem_region_local_alloc(n+1);
        memcpy(copy, text, n);
        copy[n] = '\0';
        return copy;
    } else {
        return text;
    }
}


unsigned long sns_parse_uhex( const char *arg, uint64_t max ) {
    char *endptr;
    errno = 0;
    unsigned long u  = strtoul( arg, &endptr, 16 );

    SNS_REQUIRE( 0 == errno, "Invalid hexadecimal value: %s (%s)\n", arg, strerror(errno) );
    SNS_REQUIRE( u <= max, "Argument %s too big\n", arg );

    return u;
}

double sns_parse_float( const char *arg ) {
    double x;
    int r = sscanf(arg, "%lf", &x );
    SNS_REQUIRE( 1 == r, "Couldn't parse floating point: %s\n", arg );
    return x;
}
