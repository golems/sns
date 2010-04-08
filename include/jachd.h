/*
 * jachd.h
 *
 *  Created on: Apr 8, 2010
 *      Author: jscholz
 */

#ifndef JACHD_H_
#define JACHD_H_


#define JOYSTICK_CHANNEL_NAME "joystick-data"

/**
 * Size for joystick channel when passing messages
 * from logitech gamepad (spacenav is smaller)
 */
#define JOYSTICK_CHANNEL_SIZE 78

/**
 * Message size params
 */
#define JACH_NBUTTONS 10
#define JACH_NAXES 6

#define SPACENAV_CHANNEL_NAME "spacenav-data"


/**
 * Message size params
 */
#define SNACH_NBUTTONS 2
#define SNACH_NAXES 6

#define SPNAV_MOTION_MAX 512

#endif /* JACHD_H_ */
