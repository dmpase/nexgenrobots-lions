/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This file contains an abstract definition of a minimal robot - no devices or hardware.
 * Inherit this class for your robot definition and provide init(), start(), and stop()
 * functions to manage any additional devices your robot requires.
 */

public abstract class NgTail {
    // hardware map and telemetry from the OpMode class
    public OpMode op_mode = null;
    public HardwareMap hardwareMap = null;

    public enum OpModeType { Unknown, Linear, Iterative }
    public OpModeType op_mode_type = OpModeType.Unknown;

    public NgTail(OpMode op)
    {
        op_mode = op;
        hardwareMap = op_mode.hardwareMap;
    }


    // initialize tail motors
    public abstract void init();
    public abstract void raise(double height);
    public abstract void stop();

    //    get color sensor values
    public static final int ALPHA = 0;
    public static final int RED   = 1;
    public static final int GREEN = 2;
    public static final int BLUE  = 3;

    public abstract double[] color();
    public abstract double   distance();

    // take a quick snooze, catch 40 winks, etc...
    public static void sleep(double sec)
    {
        long ms = (long)(sec * 1000);
        try {
            Thread.sleep(ms);
        } catch (Exception e) {
            ;
        }
    }
}
