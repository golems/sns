/**
 * Demo/test program that will send a waypoint path meant for a single UR5 to
 * be read in by a trajectory blender.
 * author: Bryce Willey
 */

#include <sns.h>
#include <sns/event.h>
#include <ach.h>


/** 3 different waypoint path positions: 0 -> (N_DOF - 1) = joints at point 1... */
/** Waypoint path with no collisions. */
double one_no_collision_positions[] = {
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0,  0.0,  0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.6,  0.0,  0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
};

/** Waypoint path that collides with the table between points 2 and 3.*/
double one_env_collision_positions[] = {
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0,  0.0,  0.0, -1.59, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.6,  0.0,  0.0,  1.59, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
};

/** Waypoint path that makes the robot collide with its own links.  */
double one_self_collision_positions[] = {
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.6,  0.0,  0.0, -1.59, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.6,  0.0,  0.0,  1.59, 0.6, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
};

/** 3 more, meant for 2 UR5s. */
double two_no_collision_positions[] = {
    0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, -1.59, 0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0,  0.0,  0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0,  0.0,  0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    1.6,  0.0,  0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        1.6,  0.0,  0.0, -1.59, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
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

    if (argc == 1 || (argc == 2 && strcmp(argv[1], "one") == 0)) {
        // Default path and scene.
        positions = one_no_collision_positions;
        n_steps = 4;
        n_dof = 12;
    } else if (argc == 2 && strcmp(argv[1], "two") == 0) {
        n_steps = 4;
        n_dof = 24;
        positions = two_env_collision_positions;
    }

    sns_init();
    sns_chan_open(&channel_path, "follow_path", NULL);
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
