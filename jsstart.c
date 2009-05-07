
#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <js/js.h>

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
}

void open_js() {
    js = js_open(js_index);
    if (js == NULL) {
        perror("js_open");
        exit(-1);
    }
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
    int trial = trial_index++; // Must be before fork.
    if ((pid = fork())) {
        // Parent
        if (pid < 0) {
            perror("fork");
            exit(-1);
        }
        return;
    }

    // Log name
    char log_file[256];
    snprintf(log_file, 256, "%s%d.log", log_prefix, trial);

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

    open_js();
    while (!wait_for_chord()) {
        spawn();
        watch();
    }

    return 0;
}


