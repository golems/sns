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

#include "config.h"

#include <stdint.h>
#include <amino.h>
#include <ach.h>

#include <amino/rx/rxtype.h>
#include <amino/rx/scenegraph.h>
#include <amino/ct/state.h>

#include "sns/motor.h"



void
sns_motor_map_in( const struct sns_motor_map *M,
                  size_t n_sub, const double *q_sub,
                  double *q_all )
{
    size_t n = AA_MIN(n_sub, M->n);
    for( size_t i = 0; i < n; i ++ ) {
        q_all[ M->id[i] ] = q_sub[i];
    }
}

void
sns_motor_map_destroy( struct sns_motor_map *m )
{
    for( size_t i = 0; i < m->n; i ++ ) {
        free(m->name[i]);
    }
    free(m->name);
    free(m->id);
    free(m);
}

struct sns_motor_map *
sns_motor_map_parse( const char *str )
{
    char *saveptr = NULL;
    size_t n_var = 0;
    size_t n_str = 0;

    /* Count vars and string length */
    if( '\0' != *str ) n_var++;
    while( '\0' != str[n_str] ) {
        if( ',' == str[n_str] ) n_var++;
        n_str++;
    }

    char buf[1 + n_str];
    strcpy(buf,str);

    struct sns_motor_map *M = AA_NEW(struct sns_motor_map);
    M->n = n_var;
    M->name = AA_NEW_AR(char*,M->n);
    M->id = AA_NEW_AR(aa_rx_config_id,M->n);

    /* tokenize string */
    {
        size_t i = 0;
        char *tok = strtok_r(buf, ",", &saveptr);
        while(tok) {
            assert( i < M->n );
            M->name[i] = strdup(tok);
            M->id[i] = AA_RX_FRAME_NONE;
            i++;
            tok = strtok_r(NULL, ",", &saveptr);
        }
    }

    return M;

}

int
sns_motor_map_fill_id( const struct aa_rx_sg *sg, struct sns_motor_map *M )
{
    M->sg = sg;
    for( size_t i = 0; i < M->n; i ++ ) {
        M->id[i] = aa_rx_sg_config_id(sg,M->name[i]);
        if( M->id[i] < 0 ) {
            SNS_LOG( LOG_ERR, "Unknown configuration variable: `%s'", M->name[i] );
            return  -1;
        }
    }
    return 0;
}
