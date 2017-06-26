/**
 * Demo/test program that will send a waypoint path to be read in by a trajectory blender.
 * author: Bryce Willey
 */

#include <sns.h>
#include <sns/event.h>
#include <ach.h>

/** Number of Waypoints. */
uint32_t N_STEPS = 4;

/** Number of degrees of freedom of the robot (the UR5 in this case.) */
uint32_t N_DOF = 12;

/** 3 different waypoint path positions: 0 -> (N_DOF - 1) = joints at point 1... */
/** Waypoint path with no collisions. */
double no_collision_positions[] = {
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0,  0.0,  0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.6,  0.0,  0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
};

/** Waypoint path that collides with the table between points 2 and 3.*/
double env_collision_positions[] = {
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0,  0.0,  0.0, -1.59, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.6,  0.0,  0.0,  1.59, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
};

/** Waypoint path that makes the robot collide with its own links.  */
double self_collision_positions[] = {
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.6,  0.0,  0.0, -1.59, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.6,  0.0,  0.0,  1.59, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
};

int main(void)
{
    ach_channel_t channel_path;

    sns_init();
    sns_chan_open(&channel_path, "follow_path", NULL);
    struct sns_msg_path_dense *path = sns_msg_path_dense_alloc(N_STEPS, N_DOF);

    /* Put the positions array into the path message. */
    for (size_t i = 0; i < N_STEPS * N_DOF; i++) {
        path->x[i] = self_collision_positions[i];
    }

    /* Setup the rest of the message. */
    struct timespec now;
    clock_gettime(ACH_DEFAULT_CLOCK, &now);
    sns_msg_set_time(&path->header, &now, (int64_t)(1e9)); // 1 sec duration.

    /* Send the dense path to the follow_path channel. */
    ach_put(&channel_path, path,
            sizeof(struct sns_msg_path_dense) - sizeof(sns_real_t) +
            sizeof(sns_real_t) * N_STEPS * N_DOF);

    aa_mem_region_local_pop(path);
}
