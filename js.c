/*
 * Copyright (c) 2009, Jon Olson
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   * Neither the name Jon Olson nor the names of its contributors
 *     may be used to endorse or promote products derived from this
 *     software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY Jon Olson ''AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL Jon Olson BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


/** \file js.c
 *  \author Jon Olson
 */

#include <poll.h>
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <dirent.h>
#include <unistd.h>
#include <sys/types.h>
#include "include/js.h"

#define MAX_JOYSTICKS 256
#define JOYSTICK_DIR "/dev/input"
#define JOYSTICK_FORMAT JOYSTICK_DIR "/js%d"

static js_t *js_alloc() {
    js_t *js;
   
    js = (js_t *)malloc(sizeof(js_t));
    if (js == NULL)
        return NULL;

    memset(js, 0, sizeof(js_t));

    return js;
}

js_t *js_open(uint8_t idx) {
    char js_name[64];

    snprintf(js_name, 63, JOYSTICK_FORMAT, idx);
    js_name[63] = '\0'; // Never useful, but we're careful folk around here

    js_t *js = js_alloc();
    if (js == NULL)
        goto fail;

    js->fd = open(js_name, O_RDONLY);
    if (js->fd < 0)
        goto fail;

    return js;

fail:

    free(js);
    return NULL;
}

js_t *js_open_first() {
    DIR *inputs;
    js_t *js = NULL;

    inputs = opendir("/dev/input");
    if (inputs == NULL)
        return NULL;

    struct dirent *input;
    do {
        input = readdir(inputs);
        if (input == NULL)
            break; // Could just be end of list, but if we get end of list
                   // without returning, that IS a failure
       
        unsigned int idx;
        if (sscanf(input->d_name, JOYSTICK_DIR "/js%u", &idx) > 0) {
            // Match
            js = js_alloc();
            if (js == NULL)
                break;

            js->fd = open(input->d_name, O_RDONLY);
            if (js->fd < 0) {
                free(js);
                break;
            } 
        }
    } while (input != NULL);    

    closedir(inputs);
    return js; // Either a valid js or NULL
}

js_event_t *js_poll_event(js_t *js) {
    static js_event_t event;
    assert(js != NULL);

    ssize_t len = read(js->fd, &event, sizeof(event));
    // I hope linux doesn't give us half the struct on EINTR, that would suck
    if (len < (int)sizeof(event))
        return NULL;    

    return &event;
}

int js_poll_state(js_t *js) {
    js_event_t *event;
    int more = 0;
    do {
        event = js_poll_event(js);

        if (event == NULL)
            return -1;
        
        switch (event->type) {
            case JS_EVENT_AXIS:
                js->state.axes[event->number] = (double)event->value / 32768.0;
                break;
            case JS_EVENT_BUTTON:
                js->state.buttons[event->number] = (uint8_t)event->value;
                break;
            default:
                break;
        }

        struct pollfd fd;
        fd.fd = js->fd;
        fd.events = POLLIN;
        fd.revents = 0;
        more = poll(&fd, 1, 0); 
    } while (more);

    return 0;
}
    

void js_close(js_t *js) {
    assert(js != NULL);

    close(js->fd);
    free(js);
}

