/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2008, Georgia Tech Research Corporation
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




#include "config.h"


#include <getopt.h>
#include <syslog.h>
#include <dlfcn.h>
#include "sns.h"

int main( int argc, char **argv ) {
    if( argc != 2 ) {
        fprintf(stderr, "USAGE: sns-trylock LOCK-FILENAME\n");
        exit(EXIT_FAILURE);
    }

    /* Open File */
    const char *file = argv[1];
    int fd = open(file, 0);

    if( fd < 0 ) {
        if( ENOENT == errno ) {
            exit(-1);
        } else {
            fprintf(stderr, "couldn't open `%s': %s",
                    file, strerror(errno));
            exit(EXIT_FAILURE);
        }
    }

    /* Test Lock */
    int r = lockf( fd, F_TEST, 0 );
    if( 0 == r ) {
        /* unlocked */
        exit(-1);
    } else if( (EACCES != errno && EAGAIN != errno) ) {
        fprintf(stderr, "couldn't testing lock `%s': %s",
                file, strerror(errno));
        exit(EXIT_FAILURE);
    } /* else daemon runs */


    /* Extract PID */
    FILE *fin = fdopen(fd, "r");
    int pid;
    fscanf(fin, "%d", &pid);
    printf("%d\n",pid);
    return 0;
}
