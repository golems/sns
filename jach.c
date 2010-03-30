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
        i = ach_create( (char*)name, frame_cnt, frame_size, &attr );
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

Somatic__Joystick* jach_receive(ach_channel_t *chan)
{
    size_t n;
    int r;
    if( 0 == chan->seq_num ) {
        r = ach_wait_last( chan, NULL, 0, &n, NULL );
    } else {
        r = ach_copy_last( chan, NULL, 0, &n );
    }

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
void jach_print(Somatic__Joystick *msg){
	int i;
	for (i=0; i<msg->axes->n_data; ++i)
		fprintf(stdout, "% 1.2lf::", msg->axes->data[i]);
	fprintf(stdout, "[");
	for (i=0; i<msg->buttons->n_data; ++i)
		fprintf(stdout, "%lld::", msg->buttons->data[i]);
	fprintf(stdout, "]\n");
}
