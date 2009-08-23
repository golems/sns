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

/** \file jsstart.c
 *  \author Jon Olson
 */


#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include "js/js.h"

#define eprintf(fmt, args...) fprintf(stderr, fmt, ## args)

unsigned int js_index = 0;
js_t *js;

pid_t pid;

unsigned int quit = 1;
unsigned char launch_chord[256];
unsigned int trial_index = 0;

char *log_prefix = "log";

char *process;
char **process_args;

void print_usage_and_quit(char *name) {
    eprintf("Usage: %s -j [index] -p <log prefix> -b <button0> -b <button1> ... <process> <args>\n", name);
    exit(-1);
}

void parse_command_line(int argc, char **argv) {
    unsigned char button;
    int opt = 0;

    // Oh error processing, where art thou?
    while (opt != -1) {
        switch ((opt = getopt(argc, argv, "+j:b:p:"))) {
            case 'b':
                button = atoi(optarg);
                launch_chord[button] = 1;
                break;
            case 'j':
                js_index = atoi(optarg);
                break;
            case 'p':
                log_prefix = optarg;
                break;
            case '?':
                print_usage_and_quit(argv[0]);
                break;
            default:
                break;
        }
    }

    if (optind >= argc)
        print_usage_and_quit(argv[0]);

    process = argv[optind++];
    process_args = &argv[optind];

    int i;
    for (i = 0; i < 256; i++)
        printf("%d: %d\n", i, launch_chord[i]);
}

void open_js() {
    js = js_open(js_index);
    if (js == NULL) {
        perror("js_open");
        exit(-1);
    }
}

void close_js() {
    js_close(js);
}

int wait_for_chord() {
    int i;
    do {
        int status = js_poll_state(js);
        if (status != 0) {
            perror("js_poll_state");
            exit(-1);
        }
        for (i = 0; i < 256; i++)
            if (js->state.buttons[i] != launch_chord[i])
                break;
    } while (i < 256);

    return 0;
}

void spawn() {
    // Look for existing logfile, increment trial index until we don't find one
    struct stat st;
    // Log name
    char log_file[256];
    do {
        snprintf(log_file, 256, "%s%d.log", log_prefix, trial_index++);
    } while (!stat(log_file, &st));
    eprintf("Using log: %s\n", log_file);

    if ((pid = fork())) {
        // Parent
        if (pid < 0) {
            perror("fork");
            exit(-1);
        }
        return;
    }

    // Child
    int fd;
    fd = creat(log_file, S_IRUSR);
    if (fd < 0) {
        eprintf("Unable to create log file: %s\n", log_file);
        exit(-1);
    }

    dup2(fd, STDOUT_FILENO);

    execvp(process, process_args);

    // We SHOULD NOT GET HERE
    perror("execv");
    exit(-1);
}

void watch() {
    int status;
    waitpid(pid, &status, 0);
    if (WIFEXITED(status))
        eprintf("Child with pid %d exited with status %d\n", pid, WEXITSTATUS(status));
    else
        eprintf("Child with pid %d exited due to signal %d\n", pid, WTERMSIG(status));
}

int main(int argc, char **argv) {
    memset(launch_chord, 0, 255);
    parse_command_line(argc, argv);

    do {
        open_js();
        wait_for_chord();
        close_js();
        spawn();
        watch();
    } while(1);

    return 0;
}


