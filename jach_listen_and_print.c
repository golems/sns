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
#include <stdarg.h>
#include <stdint.h>

#include <somatic.h>
#include <ach.h>

#include <somatic/util.h>
#include <somatic.pb-c.h>
#include <somatic/msg/joystick.h>

#include "include/jachd.h"

/* ----------- */
/* GLOBAL VARS */
/* ----------- */


/* ---------- */
/* ARGP Junk  */
/* ---------- */
/* Option Vars */
static int opt_verbosity = 0;
static int opt_create = 0;
static const char *opt_ach_chan = JOYSTICK_CHANNEL_NAME;

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
    case 0:
        break;
    }
    return 0;
}


/* --------------------- */
/* FUNCTION DECLARATIONS */
/* --------------------- */


/* ---- */
/* MAIN */
/* ---- */
int main( int argc, char **argv ) {

  argp_parse (&argp, argc, argv, 0, NULL, NULL);

  // install signal handler
  somatic_sighandler_simple_install();

  if (opt_create == 1)
	  somatic_create_channel(opt_ach_chan, 10, 8);

  // Open the ach channel for the spacenav data
  ach_channel_t *chan = somatic_open_channel(opt_ach_chan);

  if( opt_verbosity ) {
      fprintf(stderr, "\n* JSD *\n");
      fprintf(stderr, "Verbosity:    %d\n", opt_verbosity);
      fprintf(stderr, "channel:      %s\n", opt_ach_chan);
      fprintf(stderr,"-------\n");
  }

  /*
   *  used "size_t size = somatic__joystick__get_packed_size(js_msg);"
   *  in jachd to find the size after packing a message
   */
  int ach_result;
  while (!somatic_sig_received) {
	  Somatic__Joystick *jach_msg = somatic_joystick_receive(chan, &ach_result, JOYSTICK_CHANNEL_SIZE, NULL, &protobuf_c_system_allocator);
	  somatic_hard_assert(ach_result == ACH_OK,"Ach wait failure\n");
	  somatic_joystick_print(jach_msg);
	  somatic__joystick__free_unpacked( jach_msg, &protobuf_c_system_allocator );
  }

  somatic_close_channel(chan);

  return 0;
}

