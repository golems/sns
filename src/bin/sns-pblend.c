/*
 * sns-pblend.cpp
 * A process that converts sns_msg_path_dense into sns_msg_motor_states through
 * parabolic blending in amino.
 * Inspired by code by Zak Kingston.
 *
 * Copyright (c) 2017 Rice University
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

#include <stdbool.h>

#include <amino.h>
#include <amino/test.h>
#include <amino/ct/state.h>
#include <amino/ct/traj.h>
#include <amino/rx/scenegraph.h>
#include <amino/rx/rx_ct.h>

#include <ach.h>
#include <ach/generic.h>
#include <ach/experimental.h>

#include <sns.h>
#include <sns/event.h>
#include <sns/motor.h>
#include <sns/path.h>
#include <ach/experimental.h>

#include <getopt.h>

#include "config.h"

/**
 * An enum for the possible results of blending a path.
 */
enum blend_status {
    OKAY, /* Blend completed okay. */
    DIVERGED, /* Blend did not complete; actual configuration diverged too much from the ideal. */
};

/**
 * The reply message sent to the path sender when the path is finished executing.
 */
struct msg_path_result {
    struct sns_msg_header header; /* the message header, contains time sent. */
    enum blend_status status; /* the returned status. */
 };

/**
 * The context needed for continually following a trajectory.
 */
struct traj_follow_cx {
    /** An amino memory region from which to allocate states and seg-lists. */
    struct aa_mem_region *reg;

    /** The channel to send motor references to (ideally the actual robot driver). */
    struct sns_motor_channel *ref_out;
    struct sns_motor_ref_set *ref_set;

    /** The channel to return the final trajectory status. */
    struct ach_channel finished_out;

    /** The channel on which motor state in being recieved. */
    struct sns_motor_channel *state_in;
    struct sns_motor_state_set *state_set;

    /** The number of configurations of the robot being controlled. */
    size_t n_q;

    /** How to send motor references to the robot driver. */
    enum sns_motor_mode mode;

    /** The proportional gain when following a trajectory. */
    double k_p;

    /** The maximum distance at a single condfiguration that the robot can diverge before halting. */
    double max_diverge;

    /** The frequency of the robot driver. Determines the duration of each ref. */
    double frequency;

    /** The blended path to follow. */
    struct aa_ct_seg_list *seg_list;

    /** The time that the execution of this path was started. */
    double start_time;

    /**
     * If true, then you have recieved different trajectory to follow.
     * Restart the start_time and follow the new trajectory.
     */
    bool new_traj;
 };

/**
 * The context needed for handling waypoint paths.
 */
struct traj_blend_cx {
    /** The channel on which path messages are being recieved. */
    struct ach_channel path_in;

    /** The limits of the robot for which the path is being blended. */
    struct aa_ct_limit *limits;

    /**
     * Used to begin the process of trajectory following after the waypoints
     * are blended.
     */
    struct traj_follow_cx *follow_cx;
};

struct aa_ct_pt_list *sns_to_amino_path(struct aa_mem_region *reg, struct sns_msg_path_dense *path);
enum ach_status handle_blend_waypoint(void *cx_, void *msg_, size_t msg_size);
enum ach_status exert_control(struct aa_ct_state *ideal, struct aa_ct_state *current, struct traj_follow_cx *cx, size_t n_q, struct timespec *now);
enum ach_status periodic_follow_state(void *cx_);

int main(int argc, char **argv)
{
    double velocity_slow = 30;
    double accel_slow = 100;
    struct traj_blend_cx blend_cx;
    struct traj_follow_cx follow_cx;

    AA_MEM_ZERO(&blend_cx, 1);
    AA_MEM_ZERO(&follow_cx, 1);

    /* Defaults: must set before we parse the input args.  */
    follow_cx.mode = SNS_MOTOR_MODE_VEL;
    follow_cx.k_p = 7.0;
    follow_cx.frequency = 110;
    follow_cx.max_diverge = 0.3;

    follow_cx.new_traj = false;
    follow_cx.seg_list = NULL;

    struct timespec period = aa_tm_sec2timespec(1 / follow_cx.frequency);

    struct sns_motor_channel *last_mc = NULL;

    char *path_channel_name = NULL;
    char *finished_channel_name = NULL;
    /* Parse options. */
    {
        int c = 0;
        opterr = 0;
        while ( (c = getopt( argc, argv, "y:u:p:w:m:k:f:h:?" SNS_OPTSTRING)) != -1) {
            switch(c) {
                SNS_OPTCASES_VERSION("sns-pblend",
                                      "Copyright (c) 2017, Rice University\n",
                                      "Bryce Willey")
                case 'y':
                    sns_motor_channel_push(optarg, &follow_cx.state_in);
                    last_mc = follow_cx.state_in;
                    break;
                case 'u':
                    sns_motor_channel_push(optarg, &follow_cx.ref_out);
                    last_mc = follow_cx.ref_out;
                    break;
                case 'p':
                    if (last_mc) {
                        last_mc->priority = atoi(optarg);
                    } else {
                        SNS_DIE("No channel specificd or priority argument.");
                    }
                    break;
                case 'w':
                    path_channel_name = optarg;
                    break;
                case 'f':
                    finished_channel_name = optarg;
                    break;
                case 'm':
                    if (strcmp(optarg, "VEL") == 0) {
                        follow_cx.mode = SNS_MOTOR_MODE_VEL;
                    } else if (strcmp(optarg, "POS") == 0) {
                        follow_cx.mode = SNS_MOTOR_MODE_POS;
                    } else {
                        SNS_DIE("Unknown motor mode: '%s'\n", optarg);
                    }
                    break;
                case 'k':
                    follow_cx.k_p = atoi(optarg);
                    break;
                case '?':
                case 'h':
                    puts ("Usage: sns-pblend -u REF_CHANNEL -y STATE_CHANNEL -w PATH_CHANNEL\n"
                                  "\n"
                                  "Options:\n"
                                  "  -y <channel>,             state input channel\n"
                                  "  -u <channel>,             reference output channel\n"
                                  "  -p <priority>,            reference/state channel priority\n"
                                  "  -w <channel>,             waypoint path input channel\n"
                                  "  -f <channel>              finished reply output channel\n"
                                  "  -m <VEL or POS>           motor control mode, default = VEL\n"
                                  "  -k <gain>                 proportional gain for the controller"
                                                            ", default = 8.0\n"
                                  "  -V,                       Print program version\n"
                                  "  -?/-h,                    display this help and exit\n"
                                  "\n"
                                  "Examples:\n"
                                  "  sns-pblend -y state -u ref -w path\n"
                                  "\n"
                                  "Report bugs to <bsw2@rice.edu>"
                    );
                    exit(EXIT_SUCCESS);
                default:
                    SNS_DIE("Unknown option: '%c'\n", c);
                    break;
            }
        }
    }

    sns_init();

    SNS_REQUIRE(follow_cx.state_in, "Need state channel");
    SNS_REQUIRE(follow_cx.ref_out, "Need reference channel");
    SNS_REQUIRE(path_channel_name != NULL, "Need path channel");
    SNS_REQUIRE(finished_channel_name != NULL, "Need finished channel");

    follow_cx.reg = aa_mem_region_local_get();

    struct aa_rx_sg *scenegraph = sns_scene_load();

    size_t state_chan_count = sns_motor_channel_count(follow_cx.state_in);
    sns_motor_ref_init(scenegraph, follow_cx.ref_out, &follow_cx.ref_set, 0, NULL);

    /* Get the robot limits. */
    size_t config_count = aa_rx_sg_config_count(scenegraph);
    follow_cx.n_q = config_count;
    const char **names = (const char **)malloc(sizeof(char *) * config_count);
    aa_rx_sg_config_names(scenegraph, config_count, names);

    struct aa_ct_limit *limits = aa_rx_ct_sg_limits(follow_cx.reg, scenegraph);

    for (size_t i = 0; i < config_count; i++) {
        /* The URs are REALLY fast, let's slow that down. */
        limits->min->dq[i] /= velocity_slow;
        limits->max->dq[i] /= velocity_slow;
        limits->min->ddq[i] /= accel_slow;
        limits->max->ddq[i] /= accel_slow;
        printf("Config %s limits [%zu]\n\t Max: q = %f, dq = %f, ddq = %f\n\t "
               "Min: q = %f, dq = %f, ddq = %f\n",
                names[i], i, limits->max->q[i], limits->max->dq[i], limits->max->ddq[i],
                             limits->min->q[i], limits->min->dq[i], limits->min->ddq[i]);
    }
    blend_cx.limits = limits;

    sns_chan_open(&blend_cx.path_in, path_channel_name, NULL);
    sns_chan_open(&follow_cx.finished_out, finished_channel_name, NULL);
    blend_cx.follow_cx = &follow_cx;

    /* Setup Event Handler. */
    struct sns_evhandler handlers[state_chan_count + 1];
    handlers[0].channel = &blend_cx.path_in;
    handlers[0].context = &blend_cx;
    handlers[0].handler = handle_blend_waypoint;
    handlers[0].ach_options = ACH_O_LAST;
    sns_motor_state_init(scenegraph, follow_cx.state_in, &follow_cx.state_set, state_chan_count, handlers + 1);

    /* Run event loop. */
    enum ach_status r =
        sns_evhandle(handlers, 2,
                     &period, periodic_follow_state, &follow_cx,
                     sns_sig_term_default,
                     ACH_EV_O_PERIODIC_TIMEOUT);

    SNS_REQUIRE( sns_cx.shutdown || (ACH_OK == r),
                "Could not handle events: %s, %s\n",
                ach_result_to_string(r), strerror(errno));

    sns_end();

    return r;
}

 /**
  * \brief The event handler for trajectory blending.
  *
  * cx_ is a traj_blend_cx, and msg_ is a waypoint path (sns_msg_path_dense).
  * Blends the waypoints and sends the first sns_msg_motor_ref to the driver.
  */
enum ach_status handle_blend_waypoint(void *cx_, void *msg_, size_t msg_size)
{
    struct traj_blend_cx *cx = (struct traj_blend_cx *)cx_;
    struct sns_msg_path_dense *msg = (struct sns_msg_path_dense *)msg_;

    fprintf(stdout, "Recieved a waypoint list.\n");

    /* Cleanup old lists. */
    if (cx->follow_cx->seg_list != NULL) {
        aa_ct_seg_list_destroy(cx->follow_cx->seg_list);
    }

    struct aa_ct_pt_list *list = sns_to_amino_path(cx->follow_cx->reg, msg);
    struct aa_ct_seg_list *segs = aa_ct_tjq_pb_generate(cx->follow_cx->reg, list, cx->limits);
    cx->follow_cx->seg_list = segs;
    cx->follow_cx->new_traj = true;

    /* On the next motor state message, the robot will begin following this trajectory. */
    return (ACH_OK);
}

/**
 * Sends a message to the finish channel in the traj_follow context and resets the seg structs.
 */
void send_finish_and_stop(struct traj_follow_cx *cx, struct timespec *now, enum blend_status status) {
    struct msg_path_result result;
    sns_msg_set_time(&result.header, now, (int64_t)(1e9));
    result.status = status;
    ach_put(&cx->finished_out, &result, sizeof(result));

    aa_ct_seg_list_destroy(cx->seg_list);
    cx->seg_list = NULL;
    cx->new_traj = false;
}

/**
 * \brief The function that follows a blended trajectory.
 *
 * cx_ is a traj_follow_cx, and msg_ is a sns_motor_state.
 * Exerts KD-control on the robot to ensure that it follows the blended
 * trajectory.
 */
enum ach_status periodic_follow_state(void *cx_)
{
    struct traj_follow_cx *cx = (struct traj_follow_cx *)cx_;
    struct aa_ct_state *latest = sns_motor_state_get(cx->state_set);

    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    double seconds = ((double) now.tv_sec + (double) now.tv_nsec / (double) 1e9L);

    if (cx->seg_list == NULL) {
        /* We haven't been sent a waypoint list yet. Just keep waiting. */
        return (ACH_OK);
    }

    size_t n_q = aa_ct_seg_list_n_q(cx->seg_list);

    /* Verify the message. */
    if (latest->n_q != n_q) {
        SNS_LOG(LOG_ERR, "Motor state config size mismatch: header %zu vs ours %zu\n",
                latest->n_q, n_q);
        return (ACH_OK);
    }

    struct aa_ct_state *ideal = aa_ct_state_alloc(cx->reg, n_q, 0);
    if (cx->new_traj == true) {
        aa_ct_seg_list_eval(cx->seg_list, ideal, 0);
        /* Don't start if we're not close to the start state yet. */
        if (aa_veq(n_q, ideal->q, latest->q, 0.05)) {
            cx->start_time = seconds;
            cx->new_traj = false;
        } else {
            /* Control yourself over there. */
            enum ach_status ret = exert_control(ideal, latest, cx, n_q, &now);
            aa_mem_region_local_pop(ideal);
            return ret;
        }
    }

    double reltime = seconds - cx->start_time;
    int r = aa_ct_seg_list_eval(cx->seg_list, ideal, reltime);

    if (reltime >= aa_ct_seg_list_duration(cx->seg_list) ||
            r == AA_CT_SEG_OUT) {
        /* We ran out of time. */
        aa_ct_seg_list_eval(cx->seg_list, ideal, aa_ct_seg_list_duration(cx->seg_list) - 0.001);
        if (aa_veq(n_q, ideal->q, latest->q, 0.05)) {
            /* Let the path sender know we're done. */
            send_finish_and_stop(cx, &now, OKAY);
            aa_mem_region_local_pop(ideal);
            return (ACH_OK);
        } else {
            /* Control yourself to the end, you're not there yet. */
            enum ach_status ret = exert_control(ideal, latest, cx, n_q, &now);
            aa_mem_region_local_pop(ideal);
            return ret;
        }
    }

    enum ach_status status = exert_control(ideal, latest, cx, n_q, &now);
    aa_mem_region_local_pop(ideal);
    return status;
}

/**
 * \brief Does either position or velocity control to the ideal state from the current state.
 */
enum ach_status exert_control(
        struct aa_ct_state *ideal,
        struct aa_ct_state *current,
        struct traj_follow_cx *cx,
        size_t n_q,
        struct timespec *now) {
    /* Check to see that the position hasn't diverged too much. */
    for (size_t i = 0; i < n_q; i++) {
        if (fabs(ideal->q[i] - current->q[i]) > cx->max_diverge) {
            printf("Diverged by %f at joint %zu\n", fabs(ideal->q[i] - current->q[i]), i);
            for (size_t j = 0; j < n_q; j++) {
                printf("[%zu]: Ideal: %f, Current: %f\n", j, ideal->q[j], current->q[j]);
            }
            send_finish_and_stop(cx, now, DIVERGED);
            return (ACH_OK);
        }
    }

    /* Send the motor reference message. */
    if (cx->mode == SNS_MOTOR_MODE_POS) {
        for (size_t i = 0; i < n_q; i++) {
            cx->ref_set->u[i] = ideal->q[i];
            cx->ref_set->meta[i].mode = cx->mode;
        }
    } else if (cx->mode == SNS_MOTOR_MODE_VEL) {
        for (size_t i = 0; i < n_q; i++) {
            cx->ref_set->u[i] = ideal->dq[i] - cx->k_p * (current->q[i] - ideal->q[i]);
            cx->ref_set->meta[i].mode = cx->mode;
        }
    } else {
        /* Bad motor mode? Write 0's to be safe. */
        printf("Bad motor mode: %d\n", cx->mode);
        for (size_t i = 0; i < n_q; i++) {
            cx->ref_set->u[i] = 0;
            cx->ref_set->meta[i].mode = SNS_MOTOR_MODE_VEL;
        }
    }

    sns_motor_ref_put(cx->ref_set, now, (int64_t) ((1e9) * 3 / cx->frequency));
    return (ACH_OK);
}

/**
 * Turns sns dense paths into amino waypoint lists.
 * reg is the amino memory region from which the waypoint list will be allocated.
 *
 * NOTE: the t0 and period fields of the path argument are not used: the given
 * path is simply blended, and motor refs are sent with the current time and
 * motor frequency instead.
 */
struct aa_ct_pt_list *sns_to_amino_path(struct aa_mem_region *reg, struct sns_msg_path_dense *path)
{
    struct aa_ct_pt_list *aa_list = aa_ct_pt_list_create(reg);

    uint32_t n_steps = path->n_steps;
    size_t n_q = path->n_dof;
    for (uint32_t i = 0; i < n_steps; i++)
    {
        struct aa_ct_state *state = aa_ct_state_alloc(reg, n_q, 0);
        AA_MEM_CPY(state->q, path->x + i * n_q, n_q);
        aa_ct_pt_list_add(aa_list, state);
    }

    return aa_list;
}
