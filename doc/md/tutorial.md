Tutorial {#tutorial}
========

[TOC]

Logging {#tutorial_logger}
================
<ol>

<li> Start sns:

            sudo sns start

</li>

<li> Tail the log file.  Note that log file placement may vary depending
   on your machine's syslog configuration.

        sudo tail -f /var/log/syslog
</li>

<li> Send a log message from the shell.

              snslog "hello"
</li>

<li> Send a log message from C:

<ol>
<li> Write to `logtest.c`:

~~~~~~~~~~{.c}
int main( int argc, char **argv ) {
    sns_init();

    SNS_LOG(LOG_NOTICE, "Hello from C");

    sns_end();

    return 0;
}
~~~~~~~~~~
</li>
<li> Compile `logtest.c`:

            gcc logtest.c `pkg-config --cflags amino` -std=gnu99 -lsns -o logtest

</li>

<li> Execute `logtest`:

            sns run -d -r logtest -- `pwd`/logtest

</li>



</ol>

</li>

<li> Stop sns

            sudo sns stop
</li>
</ol>

Using the Simulator {#tutorial_simulator}
===================

1. Build the Baxter demos for Amino

        cd amino && ./configure --enable-demo-baxter && make

2. Start sns

        sns start

3. Create Channels

        ach mk state && ach mk ref

4. Start the simulator.  Note the path to the Baxter plugin in the
   Amino demos.

        sns-ksim -o state -i ref \
                 -s  amino/demo/urdf/baxter/.libs/libamino_baxter.so  -n baxter


5. Set a position reference

        snsref -p ref --  0 -1 0 0  0 0 0 0 0 0 0 0 0 0 0

6. Set a velocity reference

        snsref -d ref --  0 -1 0 0  0 0 0 0 0 0 0 0 0 0 0

7. Stop sns

        sns stop

Run a daemon in the background {#tutorial_background}
==============================

1. Start sns:

        sns start

2. We will run the simulator from the previous tutorial in the background.

        sns run -d -r bg-ksim -- \
            sns-ksim -o state -i ref \
                     -s  amino/demo/urdf/baxter/.libs/libamino_baxter.so  -n baxter

3. Kill the simulator:

        sns kill bg-ksim

4. Stop sns

        sns stop
