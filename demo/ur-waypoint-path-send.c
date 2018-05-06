/**
 * Demo/test program that will send a waypoint path meant for a single UR5 to
 * be read in by a trajectory blender.
 * author: Bryce Willey
 */

#include "config.h"

#include <sns.h>
#include <sns/event.h>
#include <ach.h>

#include <getopt.h>


/** 3 different waypoint path positions: 0 -> (N_DOF - 1) = joints at point 1... */
/** Waypoint path with no collisions. */
double one_no_collision_positions[] = {
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.59,  0.5, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
};

/** Waypoint path that collides with the table between points 2 and 3.*/
double one_env_collision_positions[] = {
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0,  0.0,  0.0, -1.59, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
};

/** Waypoint path that makes the robot collide with its own links.  */
double one_self_collision_positions[] = {
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.6,  0.0,  0.0, -1.59, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
};

/** 3 more, meant for 2 UR5s. */
double two_no_collision_positions[] = {
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    
    0.0, -1.59, 0.5, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.59, 0.5, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
};

double two_env_collision_positions[] = {
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,

    0.0,  0.0,  0.0, -1.59, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0,  0.0,  0.0, -1.59, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,

    1.6,  0.0,  0.0,  1.59, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.6,  0.0,  0.0,  1.59, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,

    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
};

int main(int argc, char **argv)
{
    double *positions;
    uint32_t n_steps; /* Number of Waypoints. */
    uint32_t n_dof; /* Number of degrees of freedom of the robot. */
    ach_channel_t channel_path;

    char *path_channel_name = NULL;
    int arm_count = 0;
    /* Parse options. */
    {
        int c = 0;
        opterr = 0;
        while ( (c = getopt( argc, argv, "w:n:h:?" SNS_OPTSTRING)) != -1) {
            switch(c) {
                SNS_OPTCASES_VERSION("ur_send_waypoints",
                                      "Copyright (c) 2017, Rice University\n",
                                      "Bryce Willey")
                case 'w':
                    path_channel_name = optarg;
                    break;
                case 'n':
                    arm_count= atoi(optarg);
                    break;
                case '?':
                case 'h':
                    puts ("Usage: sns-pblend -w PATH_CHANNEL -n NUM_ARMS\n"
                                  "\n"
                                  "Options:\n"
                                  "  -w <channel>,             waypoint path output channel\n"
                                  "  -n <num arms>             number of arms to control\n"
                                  "  -V,                       Print program version\n"
                                  "  -?/-h,                    display this help and exit\n"
                                  "\n"
                                  "Examples:\n"
                                  "  sns-pblend -w follow_path -n 1\n"
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
    SNS_REQUIRE(path_channel_name, "Need path channel");
    SNS_REQUIRE(arm_count != 0, "Need arm count.");
    n_steps = 3;
    if (arm_count == 1) {
        positions = one_no_collision_positions;
        n_dof = 12;
    } else if (arm_count == 2) {
        positions = two_no_collision_positions;
        n_dof = 24;
    } else {
        printf("We can't control %d arms\n", arm_count);
        exit(1);
    }

    sns_chan_open(&channel_path, path_channel_name, NULL);
    struct sns_msg_path_dense *path = sns_msg_path_dense_alloc(n_steps, n_dof);

    /* Put the positions array into the path message. */
    for (size_t i = 0; i < n_steps * n_dof; i++) {
        path->x[i] = positions[i];
    }

    /* Setup the rest of the message. */
    struct timespec now;
    clock_gettime(ACH_DEFAULT_CLOCK, &now);
    sns_msg_set_time(&path->header, &now, (int64_t)(1e9)); // 1 sec duration.

    /* Send the dense path to the follow_path channel. */
    ach_put(&channel_path, path,
            sizeof(struct sns_msg_path_dense) - sizeof(sns_real_t) +
            sizeof(sns_real_t) * n_steps * n_dof);

    aa_mem_region_local_pop(path);
}
