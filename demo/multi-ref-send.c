#include <sns.h>

#include <amino/rx/rxtype.h>
#include <amino/rx/scenegraph.h>
#include <amino/rx/scene_plugin.h>

#include <sns/motor.h>

int main (void)
{
    sns_init();

    /* Load Scene Plugin */
    struct aa_rx_sg *scenegraph = sns_scene_load();
    aa_rx_sg_init(scenegraph);

    /* Initialize Reference Set */
    struct sns_motor_channel *ref_chan = NULL;
    struct sns_motor_ref_set *ref_set = NULL;
    sns_motor_channel_push("ref_left", &ref_chan);
    sns_motor_channel_push("ref_right", &ref_chan);
    sns_motor_ref_init(scenegraph,
                       ref_chan, &ref_set,
                       0, NULL );

    /* Populate References */
    size_t n_q = aa_rx_sg_config_count(scenegraph);
    for( size_t i = 0; i < n_q; i ++ ) {
        ref_set->u[i] = 0;
        ref_set->meta[i].mode = SNS_MOTOR_MODE_VEL;
    }
    ref_set->u[ aa_rx_sg_config_id(scenegraph, "left_s0") ] = -1;
    ref_set->u[ aa_rx_sg_config_id(scenegraph, "right_s0") ] = -1;

    /* Send Message(s) */
    struct timespec now;
    clock_gettime( ACH_DEFAULT_CLOCK, &now );
    sns_motor_ref_put( ref_set, &now, 1e9 );

    /* End */
    sns_end();
}
