/*
 * js_smm.c
 *
 *  Created on: Mar 29, 2010
 *      Author: jscholz
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>

#include <somatic.h>
#include <ach.h>

#include <somatic/util.h>
#include <somatic.pb-c.h>

#include "include/js_smm.h"


/*
 * Allocate a Somatic__Joystick message
 */
int somatic_joystick_allocate_msg(Somatic__Joystick *msg, size_t n_axes, size_t n_buttons)
{
	somatic__joystick__init(msg);

	msg->axes = SOMATIC_NEW(Somatic__Vector);
	somatic__vector__init(msg->axes);
	msg->axes->data = SOMATIC_NEW_AR(double, n_axes);
	msg->axes->n_data = (size_t)n_axes;

	msg->buttons = SOMATIC_NEW(Somatic__Ivector);
	somatic__ivector__init(msg->buttons);
	msg->buttons->data = SOMATIC_NEW_AR(int64_t, n_buttons);
	msg->buttons->n_data = (size_t) n_buttons;

	//TODO: add return code
	return (0);
}

int somatic_joystick_free_msg(Somatic__Joystick *msg)
{
	free(msg->axes->data);
	free(msg->buttons->data);
	free(msg->axes);
	free(msg->buttons);
	//free(msg);

	return(0);
}

/**
 * Block, waiting for a mouse event
 */
void jach_read_to_msg(Somatic__Joystick *msg, js_t *js)
{
	int status = js_poll_state( js );
	somatic_hard_assert( status == 0, "Failed to poll joystick\n");

	int i;
	for( i = 0; i < JACH_NAXES; i++ )
		msg->axes->data[i] = js->state.axes[i];

	for( i = 0; i < JACH_NBUTTONS; i++ )
		msg->buttons->data[i] = (int64_t)js->state.buttons[i];
}

/**
 * Block, waiting for a mouse event
 */
void snach_read_to_msg(Somatic__Joystick *msg, spnav_event *spnevent)
{
	spnav_wait_event(spnevent);

	if (spnevent->type == SPNAV_EVENT_MOTION) {
			msg->axes->data[0] = (double)spnevent->motion.x /  SPNAV_MOTION_MAX;
			msg->axes->data[1] = (double)spnevent->motion.y /  SPNAV_MOTION_MAX;
			msg->axes->data[2] = (double)spnevent->motion.z /  SPNAV_MOTION_MAX;
			msg->axes->data[3] = (double)spnevent->motion.rx / SPNAV_MOTION_MAX;
			msg->axes->data[4] = (double)spnevent->motion.ry / SPNAV_MOTION_MAX;
			msg->axes->data[5] = (double)spnevent->motion.rz / SPNAV_MOTION_MAX;
	}
	else if (spnevent->type == SPNAV_EVENT_BUTTON) {
		if (spnevent->button.bnum == 0) {
			msg->buttons->data[0] = (int64_t)spnevent->button.press;
		}
		else if (spnevent->button.bnum == 1)  {
			msg->buttons->data[1] = (int64_t)spnevent->button.press;
		}
		else {
			//TODO remove this once it's tested:
			printf("Shouldn't be here\n");
			exit(-1);
		}
	}
}

int somatic_joystick_publish(Somatic__Joystick *msg, ach_channel_t *chan)
{
	int r = SOMATIC_PACK_SEND( chan, somatic__joystick, msg );
	somatic_hard_assert( ACH_OK == r, "Failed to send message: %s\n", ach_result_to_string( r ) );

	return(r);
}

Somatic__Joystick* somatic_joystick_receive(ach_channel_t *chan)
{
    size_t n;
    int r;
//    if( 0 == chan->seq_num ) {
        r = ach_wait_last( chan, NULL, 0, &n, NULL );
//        printf("got here\n");
//    } else {
//        r = ach_copy_last( chan, NULL, 0, &n );
//    }

    somatic_hard_assert( ACH_OVERFLOW == r, "Unknown error: %s\n",
                         ach_result_to_string( r ) );
    uint8_t buf[n];
    size_t nread;
    r = ach_copy_last( chan, &buf[0], n, &nread );
    somatic_hard_assert( n == nread &&
                         (ACH_OK == r || ACH_MISSED_FRAME == r || ACH_STALE_FRAMES == r),
                         "EZ fail: %s\n", ach_result_to_string( r ) );

    return somatic__joystick__unpack( &protobuf_c_system_allocator, n, buf );
}

/*
 * Print the contents of a Somatic__Joystick message
 */
void somatic_joystick_print(Somatic__Joystick *msg){
	int i;
	for (i=0; i<msg->axes->n_data; ++i)
		fprintf(stdout, "% 1.2lf::", msg->axes->data[i]);
	fprintf(stdout, "[");
	for (i=0; i<msg->buttons->n_data; ++i)
		fprintf(stdout, "%lld::", msg->buttons->data[i]);
	fprintf(stdout, "]\n");
}
