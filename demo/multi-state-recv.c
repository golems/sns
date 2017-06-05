#include <sns.h>
#include <sns/event.h>
#include <ach/experimental.h>

#include <amino/rx/rxtype.h>
#include <amino/rx/scenegraph.h>
#include <amino/rx/scene_plugin.h>

#include <sns/motor.h>
#include <amino/ct/state.h>

/* Callback function for the event loop */
static enum ach_status
periodic( void *cx );

int main (void)
{
    sns_init();

    /* Load Scene Plugin */
    struct aa_rx_sg *scenegraph = sns_scene_load();
    aa_rx_sg_init(scenegraph);

    /* Initialize State Set */
    struct sns_motor_channel *state_chan = NULL;
    struct sns_motor_state_set *state_set = NULL;
    struct sns_evhandler handlers[2];
    sns_motor_channel_push("state_left", &state_chan);
    sns_motor_channel_push("state_right", &state_chan);
    sns_motor_state_init(scenegraph,
                         state_chan, &state_set,
                         2, handlers);


    /* Run event loop */
    double frequency = 10;
    struct timespec period = aa_tm_sec2timespec( 1 / frequency );
    enum ach_status r =
        sns_evhandle( handlers, 2,
                      &period, periodic, state_set,
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
periodic( void *cx_ )
{
    struct sns_motor_state_set *state_set = (struct sns_motor_state_set *)cx_;
    struct aa_ct_state *state = sns_motor_state_get(state_set);
    for( size_t i = 0; i < state->n_q; i ++ ) {
        printf("%f\t", state->q[i]);
    }
    printf("\n");
    return ACH_OK;
}
