/* -*- mode: C; c-basic-offset: 4 -*- */
/* ex: set shiftwidth=4 tabstop=4 expandtab: */
/*
 * Copyright (c) 2014, Georgia Tech Research Corporation
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

#include "config.h"

#include <inttypes.h>
#include <getopt.h>
#include <unistd.h>
#include "sns.h"

static const char *opt_file = NULL;


static void posarg( char *arg, int i ) {
    if( 0 == i ) {
        opt_file = strdup(arg);
    } else {
        fprintf(stderr, "Invalid arg: %s\n", arg);
        exit(EXIT_FAILURE);
    }
}

int main( int argc, char **argv ) {

    /*-- Parse Options --*/
    {
        int i = 0;
        for( int c; -1 != (c = getopt(argc, argv, "V?")); ) {
            switch(c) {
                SNS_OPTCASES
            case 'V':   /* version     */
                puts( "snsrec " PACKAGE_VERSION "\n"
                      "\n"
                      "Copyright (c) 2013, Georgia Tech Research Corporation\n"
                      "This is free software; see the source for copying conditions.  There is NO\n"
                      "warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.\n"
                      "\n"
                      "Written by Neil T. Dantam"
                    );
                exit(EXIT_SUCCESS);
            case '?':   /* help     */
                puts( "Usage: snsreced [OPTIONS...] file\n"
                      "Manipulate recorded SNS messages\n"
                      "\n"
                      "Options:\n"
                      "  -?,                          Give program help list\n"
                      "  -V,                          Print program version\n"
                      "\n"
                      "Report bugs to <ntd@gatech.edu>"
                    );
                exit(EXIT_SUCCESS);
                break;
            default:
                posarg( optarg, i++ );
            }
        }
        while( optind < argc ) {
            posarg(argv[optind++], i++);
        }
    }


    FILE *fin = fopen(opt_file,"r");
    FILE **fout;
    struct aa_mem_region *reg = aa_mem_region_local_get();
    int first = 1;
    size_t n_real;
    size_t lineno = 1;
    double t0;

    while( !feof(fin) ) {
        // read line
        char *line = aa_io_getline(fin,reg);
        if( NULL == line ) continue;

        char *line2 = aa_io_skipblank(line);
        if('\0' == *line2 || AA_IO_ISCOMMENT(*line2)) continue;

        double *X;
        size_t n = aa_io_d_parse(line2, reg, &X, NULL );
        if( first ) {
            if( n <= 1 ) {
                fprintf(stderr,"Too few elements\n");
                exit(EXIT_FAILURE);
            }
            first = 0;
            n_real = n;
            fout = (FILE**)malloc((n-1)*sizeof(FILE*));
            for( size_t i = 0; i < n-1; i ++ ) {
                char *name = aa_mem_region_printf( reg, "%lu.dat", i );
                fout[i] = fopen(name,"w");
            }
            t0 = X[0];
        } else if( n != n_real ) {
            fprintf(stderr,"Error on line %lu: n=%lu, expected %lu\n", lineno, n, n_real);
            exit(EXIT_FAILURE);
        }

        // write line
        for( size_t i = 0; i < n-1; i ++ ) {
            fprintf(fout[i],"%f %f\n",X[0]-t0, X[i+1]);
        }
        aa_mem_region_release(reg);
        lineno++;
    }


    return 0;
}
