Programming Tutorial {#tutorial_prog}
==============


m4_changequote(`[[', `]]')

[TOC]

Run the Simulator {#tutorial_prog_sim}
=================

We will use the simulator from the [basic tutorial] (@ref tutorial) to
demonstrate the SNS API.


1. Start sns:

        sudo sns start

2. Create new channels:

        ach mk state_left
        ach mk state_right
        ach mk state_head
        ach mk ref_left
        ach mk ref_right
        ach mk ref_head

3. Define environment variables for the scene plugin:

        export SNS_SCENE_PLUGIN=/path/to/amino/demo/urdf/baxter/.libs/libamino_baxter.so
        export SNS_SCENE_NAME=baxter

4. Define environment variables for axes to remap:

        export SNS_CHANNEL_MAP_state_left="left_s0,left_s1,left_e0,left_e1,left_w0,left_w1,left_w2"
        export SNS_CHANNEL_MAP_state_right="right_s0,right_s1,right_e0,right_e1,right_w0,right_w1,right_w2"
        export SNS_CHANNEL_MAP_state_head="head_pan"

        export SNS_CHANNEL_MAP_ref_left="$SNS_CHANNEL_MAP_state_left"
        export SNS_CHANNEL_MAP_ref_right="$SNS_CHANNEL_MAP_state_right"
        export SNS_CHANNEL_MAP_ref_head="$SNS_CHANNEL_MAP_state_head"

5. Restart the simulator.  It will find the remapping parameters in
   the environment variables.

        sns run -d -r bg-ksim -- \
            sns-ksim -y state_left  -u ref_left \
                     -y state_right -u ref_right \
                     -y state_head  -u ref_head


Send a motor reference {#tutorial_prog_ref}
======================

The following program will send a reference velocity to the
`head_pan` joint of the robot:

~~~~~~~~~~~~~~~~~~~~~~{.c}
m4_include(demo/motor-ref.c)
~~~~~~~~~~~~~~~~~~~~~~

Receive motor state {#tutorial_prog_state}
===================

The following program will receive state messages and print the
position to `STDOUT`.

~~~~~~~~~~~~~~~~~~~~~~{.c}
m4_include(demo/motor-state.c)
~~~~~~~~~~~~~~~~~~~~~~

Multiple Robots / Multiplexing {#tutorial_prog_multi}
==============================

To control multiple robots, e.g., a left and right arm, you may need
to send/receive state/reference messages on multiple channels.


Send Multiplexed Reference  {#tutorial_prog_multi_ref_send}
--------------------------

~~~~~~~~~~~~~~~~~~~~~~{.c}
m4_include(demo/multi-ref-send.c)
~~~~~~~~~~~~~~~~~~~~~~

Receive Multiplexed State  {#tutorial_prog_multi_state_recv}
-------------------------

~~~~~~~~~~~~~~~~~~~~~~{.c}
m4_include(demo/multi-state-recv.c)
~~~~~~~~~~~~~~~~~~~~~~
