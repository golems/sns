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
#include "sns/event.h"



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

AA_API void
sns_motor_map_state_out( const struct aa_ct_state *state,
                         const struct sns_motor_map *M,
                         const struct timespec *now, int64_t dur_ns,
                         struct ach_channel *channel )
{
    /* allocate */
    struct sns_msg_motor_state *msg;
    uint32_t n_msg = (uint32_t) (M ?  M->n : state->n_q);
    msg = sns_msg_motor_state_local_alloc(n_msg);

    /* TODO: make pos and vel optional in state message */

    /* fill values */
    double *pos = sns_msg_motor_state_pos(msg);
    double *vel = sns_msg_motor_state_vel(msg);
    int incpos = (int)sns_msg_motor_state_incpos(msg);
    int incvel = (int)sns_msg_motor_state_incvel(msg);
    int n_q = (int)state->n_q;

    if(M) {
        for( size_t i = 0; i < M->n; i ++ ) {
            aa_rx_config_id j = M->id[i];
            *pos = state->q[j];
            *vel = state->dq[j];
            pos += incpos;
            vel += incpos;
        }
    } else {
        if(state->q) cblas_dcopy( n_q, state->q, 1, pos, incpos );
        if(state->dq) cblas_dcopy( n_q, state->dq, 1, vel, incvel );
    }

    sns_msg_set_time( &msg->header, now, dur_ns );

    /* send message */
    sns_msg_motor_state_put(channel, msg);

    /* deallocation */
    aa_mem_region_local_pop(msg);
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

void
sns_motor_channel_push( const char *name, struct sns_motor_channel **phead )
{
    struct sns_motor_channel *mc = AA_NEW0(struct sns_motor_channel);
    mc->name = name;
    mc->next = *phead;
    *phead = mc;
}

void
sns_motor_channel_put( struct sns_motor_channel *list, const struct aa_ct_state *state,
                       const struct timespec *t, int64_t dur_ns )
{

    while(list) {
        sns_motor_map_state_out( state, list->map,
                                 t, dur_ns,
                                 &list->channel );
        list = list->next;
    }
}

void
sns_motor_channel_parse_map( struct sns_motor_channel *mc, const char *mapstr )
{
    mc->map = sns_motor_map_parse(mapstr);
}

void
sns_motor_channel_init( struct sns_motor_channel *list, const struct aa_rx_sg *scenegraph )
{
    while( list ) {
        sns_chan_open( &list->channel, list->name, NULL );

        if( !list->map ) {
            static const char base[] = "SNS_CHANNEL_MAP_";
            char buf[ strlen(base) + strlen(list->name) + 1 ];
            strcpy(buf,base);
            strcat(buf,list->name);
            char *e = getenv(buf);
            if(e) {
                SNS_LOG(LOG_NOTICE,"Channel `%s' map: `%s'\n", list->name, e);
                list->map = sns_motor_map_parse(e);
                assert(list->map);
            }
        }

        if( list->map ) {
            sns_motor_map_fill_id( scenegraph, list->map );
        }
        list = list->next;
    }
}


AA_API void
sns_motor_ref_fill ( const struct sns_msg_motor_ref *msg,
                     struct sns_motor_ref_elt *ref_elt )
{
    size_t n = ref_elt->n;

    /* Validate */
    if( msg->header.n != n ) {
        SNS_LOG(LOG_ERR, "Mistmached element count in reference message: %lu, wanted %d\n",
                msg->header.n, n);
    } else {
        /* fill */
        /* meta.priority set during initialization */
        ref_elt->meta.time = sns_msg_get_time(&msg->header);
        ref_elt->meta.expiration = sns_msg_get_expiration(&msg->header);
        ref_elt->meta.mode = msg->mode;
        AA_MEM_CPY(ref_elt->u, msg->u, n);
    }
}

static int is_expired( struct timespec now, struct timespec expiration )
{
    if( aa_tm_cmp(now, expiration) < 0 ) return 0;
    else return 1;
}

static void collate1( const struct timespec *now,
                      const struct sns_motor_ref_elt *e,
                      struct sns_motor_ref_set *set,
                      double u,
                      size_t j )
{
    struct sns_motor_ref_meta *smeta = set->meta + j;
    long cu = aa_tm_cmp(e->meta.time,smeta->time);
    int pu = e->meta.priority, ps = smeta->priority;

    if( is_expired(*now,smeta->expiration) ||  /* old value expired */
        (pu > ps ) || /* new priority greater */
        ((pu == ps) && (cu > 0)) ) /* same priority, newer message */
    {
        set->u[j] = u;
        *smeta = e->meta;
    }
}


AA_API void
sns_motor_ref_collate ( const struct timespec *now,
                        struct sns_motor_ref_set *set )
{
    size_t n_elt = set->n_elt;
    const struct sns_motor_ref_elt *elts = set->elt;
    for( size_t i = 0; i < n_elt; i ++ ) {
        const struct sns_motor_ref_elt *e = elts + i;

        /* check if expired */
        if( is_expired(*now, e->meta.expiration) < 0 ) continue;
        /* else not expired */

        struct sns_motor_map *map = sns_motor_ref_elt_map(e);
        if( map ) {
            /* remap */
            for( size_t je = 0; je < e->n; je ++ ) {
                size_t js = (size_t)map->id[je];
                collate1(now, e, set, e->u[je], js);
            }
        } else {
            /* copy */
            for( size_t j = 0; j < e->n && j < set->n_q; j ++ ) {
                collate1(now, e, set, e->u[j], j);
            }
        }
    }
}

size_t
sns_motor_channel_count( struct sns_motor_channel *list )
{
    size_t n = 0;
    while(list) {
        n++;
        list = list->next;
    }
    return n;
}

AA_API struct sns_motor_ref_elt *
sns_motor_ref_elt_init( const struct aa_rx_sg *scenegraph,
                        struct sns_motor_channel *list )
{
    size_t n_list = sns_motor_channel_count(list);
    size_t n_q_all = aa_rx_sg_config_count(scenegraph);
    struct sns_motor_ref_elt *elts = AA_NEW0_AR(struct sns_motor_ref_elt, n_list);
    size_t i = 0;
    while(list) {
        struct sns_motor_ref_elt *e = elts + i;
        e->n = list->map ? list->map->n : n_q_all;
        e->u = AA_NEW0_AR(double,e->n);
        e->meta.priority = list->priority;
        e->channel = list;

        list = list->next;
        i++;
    }
    return elts;

}

static enum ach_status
ref_handler( void *cx_, void *msg_, size_t frame_size )
{
    struct sns_msg_motor_ref *msg = (struct sns_msg_motor_ref *)msg_;
    struct sns_motor_ref_elt *e = (struct sns_motor_ref_elt*)cx_;

    if( sns_msg_motor_ref_check_size(msg,frame_size) ) {
        SNS_LOG(LOG_ERR, "Mismatched message size on channel `%s'\n",
                e->channel->name);
    } else {
        sns_motor_ref_fill(msg, e);
    }
    return ACH_OK;
}

AA_API void
sns_motor_ref_init( const struct aa_rx_sg *scenegraph,
                    struct sns_motor_channel *list,
                    struct sns_motor_ref_set **ref_set,
                    size_t n_handlers, struct sns_evhandler *handlers )
{
    /* Init channels */
    sns_motor_channel_init(list, scenegraph);

    /* Init ref set */
    *ref_set = AA_NEW0( struct sns_motor_ref_set );

    (*ref_set)->scenegraph = scenegraph;
    (*ref_set)->n_q = aa_rx_sg_config_count(scenegraph);
    (*ref_set)->u = AA_NEW0_AR(double, (*ref_set)->n_q);
    (*ref_set)->meta = AA_NEW0_AR(struct sns_motor_ref_meta, (*ref_set)->n_q);

    (*ref_set)->n_elt = sns_motor_channel_count(list);
    (*ref_set)->elt = sns_motor_ref_elt_init(scenegraph,list);

    /* Init Handlers */
    for( size_t i = 0; i < n_handlers && i < (*ref_set)->n_elt; i ++ )
    {
        struct sns_motor_ref_elt *e = (*ref_set)->elt + i;
        handlers[i].channel = &e->channel->channel;
        handlers[i].context = e;
        handlers[i].handler = ref_handler;
        handlers[i].ach_options = ACH_O_LAST;
    }
}

AA_API struct sns_motor_map *
sns_motor_ref_elt_map( const struct sns_motor_ref_elt *e )
{
    return e->channel->map;
}

AA_API size_t
sns_motor_ref_elt_config_count( const struct sns_motor_ref_elt *e )
{
    return e->n;
}


AA_API const struct aa_rx_sg *
sns_motor_map_sg(struct sns_motor_map *map)
{
    return map->sg;
}
