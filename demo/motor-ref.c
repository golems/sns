#include <sns.h>

int main (void)
{
    sns_init();

    /* Open Channel */
    struct ach_channel channel;
    sns_chan_open(&channel, "ref_head", NULL);

    /* Populate Message */
    struct sns_msg_motor_ref *msg = sns_msg_motor_ref_local_alloc(1);
    struct timespec now;
    clock_gettime( ACH_DEFAULT_CLOCK, &now );
    sns_msg_set_time( &msg->header, &now, (int64_t)(1e9) ); /* 1 second duration */
    msg->mode = SNS_MOTOR_MODE_VEL;
    msg->u[0] = 1;

    /* Send Message */
    sns_msg_motor_ref_put(&channel,msg);

    /* Release Message */
    aa_mem_region_local_pop(msg);

    /* End */
    sns_end();
}
