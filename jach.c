/*
 * jach.c
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


/*
 * Creates an ach channel (so you don't have to use achtool)
 */
int jach_create_channel(const char *name, size_t frame_cnt, size_t frame_size) {

    int i;
    {
        ach_create_attr_t attr;
        ach_create_attr_init(&attr);
        i = ach_create( name, frame_cnt, frame_size, &attr );
    }
    somatic_hard_assert( ACH_OK == i, "Error creating channel: %s\n",
                             ach_result_to_string( i ) );

    return i;
}

/*
 * Opens a joystick channel
 */
ach_channel_t* jach_open(const char *name)
{
	ach_channel_t *chan = SOMATIC_NEW( ach_channel_t );
    int r = ach_open( chan, name, NULL );
    somatic_hard_assert( ACH_OK == r, "Error opening channel: %s\n",
                         ach_result_to_string( r ) );

    r = ach_flush( chan );
    somatic_hard_assert( ACH_OK == r, "Error flushing channel: %s\n",
                         ach_result_to_string( r ) );

	return(chan);
}

/*
 * Closes a joystick channel
 */
int jach_close(ach_channel_t *chan)
{
	int r = ach_close( chan );
	somatic_hard_assert( ACH_OK == r, "Error closing channel: %s\n",
	                         ach_result_to_string( r ) );

	return(r);
}

/*
 * Allocate a Somatic__Joystick message for spacenav data
 */
int jach_allocate_msg(Somatic__Joystick *msg, size_t n_axes, size_t n_buttons)
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

int jach_publish(Somatic__Joystick *msg, ach_channel_t *chan)
{
	int r = SOMATIC_PACK_SEND( chan, somatic__joystick, msg );
	somatic_hard_assert( ACH_OK == r, "Failed to send message: %s\n", ach_result_to_string( r ) );

	//TODO: add return code
	return(0);
}

