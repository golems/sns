/* -*- mode: C; c-basic-offset: 4  -*- */
/* ex: set shiftwidth=4 expandtab: */
/*
 * Copyright (c) 2013, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Neil T. Dantam <ntd@gatech.edu>
 * Georgia Tech Humanoid Robotics Lab
 * Under Direction of Prof. Mike Stilman <mstilman@cc.gatech.edu>
 *
 *
 * This file is provided under the following "BSD-style" License:
 *
 *
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 *
 */


#ifndef GAMEPAD_H
#define GAMEPAD_H

// Button and axis definitions for a gamepad
// Taken from Logitech F310
// TODO: D-Pad detected as axes, really buttons

enum gamepad_buttons {
    GAMEPAD_BUTTON_A =           0x001,
    GAMEPAD_BUTTON_B =           0X002,
    GAMEPAD_BUTTON_X =           0X004,
    GAMEPAD_BUTTON_Y =           0X008,
    GAMEPAD_BUTTON_LB =          0X010,
    GAMEPAD_BUTTON_RB =          0X020,
    GAMEPAD_BUTTON_BACK =        0X040,
    GAMEPAD_BUTTON_START =       0X080,
    GAMEPAD_BUTTON_LOGO =        0X200,
    GAMEPAD_BUTTON_LEFT_STICK =  0X200,
    GAMEPAD_BUTTON_RIGHT_STICK = 0X400
};

enum gamepad_axis {
    GAMEPAD_AXIS_LX = 0,
    GAMEPAD_AXIS_LY = 1,
    GAMEPAD_AXIS_LT = 2,
    GAMEPAD_AXIS_RX = 3,
    GAMEPAD_AXIS_RY = 4,
    GAMEPAD_AXIS_RT = 5,
    GAMEPAD_AXIS_DX = 6,
    GAMEPAD_AXIS_DY = 7,
};

#endif // GAMEPAD_H
