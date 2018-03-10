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

public abstract class NgDrive {
    // hardware map and telemetry from the OpMode class
    public OpMode op_mode = null;
    public HardwareMap hardwareMap = null;

    public NgDrive(OpMode op)
    {
        op_mode = op;
        hardwareMap = op_mode.hardwareMap;
    }

    public static enum Surface {FIELD, TABLE}

    public static final int PORT_BOW = 0;
    public static final int STBD_BOW = 1;
    public static final int STBD_AFT = 2;
    public static final int PORT_AFT = 3;


    // initialize drive motors
    public abstract void init();

    // stop drive motors
    public abstract void stop();


    // linear (blocking) op mode functions - autonomous op mode
    //    blocking heading change
    public abstract void set_new_heading(double angle, double power, int tolerance, Surface surface);

    //    blocking position change
    public abstract void set_new_position(double bearing, double range, double power, int tolerance, Surface surface);


    // iterative (non-blocking) op mode functions - driver op mode
    //    non-blocking heading change
    public abstract void turn(double power);

    //    non-blocking position change
    public abstract void move(double bearing, double rotation, double power);

    //    set drive motor powers independently
    public abstract void set_drive_power(double port_bow, double stbd_bow, double stbd_aft, double port_aft);

    //    get current drive motor encoder values
    public abstract int[] get_drive_encoders();

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
