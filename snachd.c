/* -*- mode: C; c-basic-offset: 2  -*- */
/*
 * Copyright (c) 2010, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *     * Redistributions of source code must retain the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials
 *       provided with the distribution.
 *     * Neither the name of the Georgia Tech Research Corporation nor
 *       the names of its contributors may be used to endorse or
 *       promote products derived from this software without specific
 *       prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY GEORGIA TECH RESEARCH CORPORATION ''AS
 * IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL GEORGIA
 * TECH RESEARCH CORPORATION BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** Author: jscholz
 */

#include <argp.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>

#include <somatic.h>
#include <somatic/daemon.h>
#include <ach.h>
#include <amino.h>

#include <somatic/util.h>
#include <somatic.pb-c.h>
#include <somatic/msg.h>

#include <spnav.h>
#include "include/jachd.h"

/* ----------- */
/* GLOBAL VARS */
/* ----------- */


/* Option Vars */
static int opt_verbosity = 0;
static int opt_create = 0;
static const char *opt_ach_chan = SPACENAV_CHANNEL_NAME;

struct snachd_cx {
  somatic_d_t d;
} CX;

/* ---------- */
/* ARGP Junk  */
/* ---------- */
static struct argp_option options[] = {
    {
        .name = "verbose",
        .key = 'v',
        .arg = NULL,
        .flags = 0,
        .doc = "Causes verbose output"
    },
    {
        .name = "channel",
        .key = 'c',
        .arg = "channel",
        .flags = 0,
        .doc = "ach channel to use (default \"spacenav-data\")"
    },
    {
        .name = "Create",
        .key = 'C',
        .arg = NULL,
        .flags = 0,
        .doc = "Create channel with specified name (off by default)"
    },
    {
        .name = NULL,
        .key = 0,
        .arg = NULL,
        .flags = 0,
        .doc = NULL
    }
};


/// argp parsing function
static int parse_opt( int key, char *arg, struct argp_state *state);
/// argp program version
const char *argp_program_version = "snachd v0.0.1";
/// argp program arguments documention
static char args_doc[] = "";
/// argp program doc line
static char doc[] = "reads from space navigator and pushes out ach messages";
/// argp object
static struct argp argp = {options, parse_opt, args_doc, doc, NULL, NULL, NULL };


static int parse_opt( int key, char *arg, struct argp_state *state) {
    (void) state; // ignore unused parameter
    switch(key) {
    case 'v':
        opt_verbosity++;
        break;
    case 'c':
        opt_ach_chan = strdup( arg );
        break;
    case 'C':
    	opt_create = 1;
    	break;
    case 0:
        break;
    }
    return 0;
}

/* ------------- */
/* Function Defs */
/* ------------- */

/**
 * Block, waiting for a mouse event
 */
void snach_read_to_msg(Somatic__Joystick *msg, spnav_event *spnevent)
{
	spnav_wait_event(spnevent);

	// Axes switch on purpose because they seem to be incorrectly labeled in driver (initial=xzyRYP)
	if (spnevent->type == SPNAV_EVENT_MOTION) {
			msg->axes->data[0] = (double)spnevent->motion.x /  SPNAV_MOTION_MAX;
			msg->axes->data[2] = (double)spnevent->motion.y /  SPNAV_MOTION_MAX;
			msg->axes->data[1] = (double)spnevent->motion.z /  SPNAV_MOTION_MAX;
			msg->axes->data[3] = (double)spnevent->motion.rx / SPNAV_MOTION_MAX;
			msg->axes->data[5] = (double)spnevent->motion.ry / SPNAV_MOTION_MAX;
			msg->axes->data[4] = (double)spnevent->motion.rz / SPNAV_MOTION_MAX;
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

/* ---- */
/* MAIN */
/* ---- */
int main( int argc, char **argv ) {

  argp_parse (&argp, argc, argv, 0, NULL, NULL);

  somatic_d_opts_t opts;
  opts.ident = "snachd";
  somatic_d_init(&CX.d, &opts );

  // install signal handler
  somatic_sighandler_simple_install();

  // spnav event
  spnav_event spnevent;

  // Open spacenav device
  int sn_r = spnav_open();
  somatic_hard_assert( sn_r == 0, "Failed to open spacenav device: %d\n", sn_r);

  // Open the ach channel for the spacenav data
  ach_channel_t chan;
  somatic_d_channel_open(&CX.d, &chan, opt_ach_chan, NULL);

  Somatic__Joystick *spnav_msg = somatic_joystick_alloc(SNACH_NAXES, SNACH_NBUTTONS);

  if( opt_verbosity ) {
      fprintf(stderr, "\n* JSD *\n");
      fprintf(stderr, "Verbosity:    %d\n", opt_verbosity);
      fprintf(stderr, "channel:      %s\n", opt_ach_chan);
      fprintf(stderr, "message size: %d\n", somatic__joystick__get_packed_size(spnav_msg) );
      fprintf(stderr,"-------\n");
  }

  while (!somatic_sig_received) {
	  snach_read_to_msg(spnav_msg, &spnevent);
	  SOMATIC_PACK_SEND( &chan, somatic__joystick, spnav_msg );
	  if( opt_verbosity )
            aa_dump_vec( stdout, spnav_msg->axes->data, spnav_msg->axes->n_data );
  }

  // Cleanup:
  somatic_d_channel_close(&CX.d, &chan );
  sn_r = spnav_close();
  somatic_hard_assert( sn_r == 0, "Failed to close spacenav device\n");

  somatic_joystick_free(spnav_msg);
  somatic_d_destroy( &CX.d );

  return 0;
}


