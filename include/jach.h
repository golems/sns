/*
 * jach.h
 *
 *  Created on: Mar 29, 2010
 *      Author: jscholz
 */

#ifndef JACH_H_
#define JACH_H_

// Creates a joystick channel
int jach_create_channel(const char *name, size_t frame_cnt, size_t frame_size);

// Open a joystick channel
ach_channel_t* jach_open(const char *name);

// Close a joystick channel
int jach_close(ach_channel_t *chan);

// Allocate a Somatic__Joystick message for spacenav data
int jach_allocate_msg(Somatic__Joystick *msg, size_t n_axes, size_t n_buttons);

// Publish message on Ach channel
int jach_publish(Somatic__Joystick *msg, ach_channel_t *chan);

// Receive a message from specified Ach channel
Somatic__Joystick* jach_receive(ach_channel_t *chan);

/*
 * Helper fn for debugging.  delete if you don't like
 * functions in header files
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

#endif /* JACH_H_ */
