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

<div>
<img src="img/ref.png" style="width:50%"></img>

<div style="text-align:center; display:block"><strong>Process Diagram</strong></div>
</div>

1. Build the Baxter demos for Amino

        cd amino && ./configure --enable-demo-baxter && make

2. Define environment variables for the scene plugin:

        export SNS_SCENE_PLUGIN=/path/to/amino/demo/urdf/baxter/.libs/libamino_baxter.so
        export SNS_SCENE_NAME=baxter

3. Start sns

        sns start

4. Create Channels

        ach mk state && ach mk ref

5. Start the simulator.  It will use the plugin defined in the
   environment variables `SNS_SCENE_PLUGIN` and `SNS_SCENE_NAME`

        sns-ksim -y state -u ref

6. Set a position reference

        snsref -p ref --  0 -1 0 0  0 0 0 0 0 0 0 0 0 0 0

7. Set a velocity reference

        snsref -d ref --  0 -1 0 0  0 0 0 0 0 0 0 0 0 0 0

8. Stop sns

        sns stop

Run a daemon in the background {#tutorial_background}
==============================

1. Start sns:

        sns start

2. We will run the simulator from the previous tutorial in the background.

        sns run -d -r bg-ksim -- sns-ksim -y state -u ref

3. Kill the simulator:

        sns kill bg-ksim

4. Stop sns

        sns stop

Teleoperate a robot {#tutorial_teleop}
===================


<div>
<img src="img/teleop.png" style="width:50%"></img>

<div style="text-align:center; display:block"><strong>Process Diagram</strong></div>
</div>

1. Make a joystick/gamepad channel:

        ach mk joystick

2. Start the joystick/gamepad driver (for a 6-axis gamepad):

        sns run -d -r joy -- sns-joyd -a 6


3. Dump the joystick channel and fiddle with the joystick to see how
   the axes and buttons are mapped on your particular hardware:

        snsdump joystick joystick

4. Start the simulator:

        sns run -d -r bg-ksim -- sns-ksim -y state -u ref

5. Start the teleoperation controller.  The `-Q` parameter controls
   which button enables the subsequently listed set of axes.  You may
   need to change the argument to `-Q` depending on your particular
   joystick hardware.

        sns-teleopd -j joystick \
                    -y state \
                    -u ref \
                    -Q 4 -m "left_s0,left_s1,left_e0,left_e1" \
                    -Q 6 -m "left_w0,left_w1,left_w2" \
                    -Q 5 -m "right_s0,right_s1,right_e0,right_e1" \
                    -Q 7 -m "right_w0,right_w1,right_w2"

6. Push one of the buttons and wiggle the joystick axes to move the
   robot.

Multiple Robots {#tutorial_remap}
==============

<div>
<img src="img/multi.png" style="width:50%"></img>

<div style="text-align:center; display:block"><strong>Process Diagram</strong></div>
</div>

If you have multiple "robots," for example, a left and right arm, you
may want to simulate these in one instance.  Compile all the robots
into a single scenegraph. Then, we will remap the state and reference
variables to split the input and output over multiple channels.

1. Create new channels:

        ach mk state_left
        ach mk state_right
        ach mk state_head
        ach mk ref_left
        ach mk ref_right
        ach mk ref_head

2. Define environment variables for axes to remap:

        export SNS_CHANNEL_MAP_state_left="left_s0,left_s1,left_e0,left_e1,left_w0,left_w1,left_w2"
        export SNS_CHANNEL_MAP_state_right="right_s0,right_s1,right_e0,right_e1,right_w0,right_w1,right_w2"
        export SNS_CHANNEL_MAP_state_head="head_pan"

        export SNS_CHANNEL_MAP_ref_left="$SNS_CHANNEL_MAP_state_left"
        export SNS_CHANNEL_MAP_ref_right="$SNS_CHANNEL_MAP_state_right"
        export SNS_CHANNEL_MAP_ref_head="$SNS_CHANNEL_MAP_state_head"

3. Restart the simulator.  It will find the remapping parameters in
   the environment variables.

        sns run -d -r bg-ksim -- \
            sns-ksim -y state_left  -u ref_left \
                     -y state_right -u ref_right \
                     -y state_head  -u ref_head

4. Send a reference to the head, then the left arm

        snsref -d ref_head  --  1
        snsref -d ref_left --  1 0 0 0 0 0 0


5. Kill the simulator:

        sns kill bg-ksim

Motor Priority {#tutorial_priority}
=============
