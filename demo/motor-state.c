#include <sns.h>
#include <sns/event.h>
#include <ach/experimental.h>

/* Callback function for the event loop */
static enum ach_status
handle_state( void *cx, void *msg, size_t msg_size );

int main (void)
{
    sns_init();

    /* Open Channel */
    struct ach_channel channel;
    sns_chan_open(&channel, "state_head", NULL);

    /* Setup Event Handler */
    struct sns_evhandler handlers[1];
    handlers[0].channel = &channel;
    handlers[0].context = NULL;
    handlers[0].handler = handle_state;
    handlers[0].ach_options = ACH_O_LAST;

    /* Run event loop */
    enum ach_status r =
        sns_evhandle( handlers, 1,
                      NULL, NULL, NULL,
                      sns_sig_term_default,
                      ACH_EV_O_PERIODIC_TIMEOUT );

    SNS_REQUIRE( sns_cx.shutdown || (ACH_OK == r),
                 "Could not handle events: %s, %s\n",
                 ach_result_to_string(r),
                 strerror(errno) );

    /* End */
    sns_end();
}

static enum ach_status
handle_state( void *cx_, void *msg_, size_t msg_size )
{
    (void)cx_;
    struct sns_msg_motor_state *msg = (struct sns_msg_motor_state *)msg_;

    /* Validate message size*/
    if( sns_msg_motor_state_check_size(msg,msg_size) ) {
        SNS_LOG(LOG_ERR, "Mismatched message size on channel\n");
    } else {
        /* Message size is ok */
        size_t inc_pos = sns_msg_motor_state_incpos(msg);
        double *pos = sns_msg_motor_state_pos(msg);
        if( pos ) {
            for( size_t i = 0; i < (size_t)msg->header.n; i ++ ) {
                printf("%f\t",pos[i*inc_pos]);
            }
        }
        printf("\n");
    }
    return ACH_OK;
}
