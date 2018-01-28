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

public abstract class GriffinRobot {
    // hardware map and telemetry from the OpMode class
    public HardwareMap hardwareMap = null;
    public Telemetry telemetry = null;

    public GriffinRobot(OpMode op_mode)
    {
        hardwareMap = op_mode.hardwareMap;
        telemetry   = op_mode.telemetry;
    }

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public abstract void init();

    /*
     * Code to run ONCE after the driver hits START
     */
    public abstract void start();

    /*
     * Code to run ONCE after the driver hits STOP
     */
    public abstract void stop();


    public static enum Command {ROTATE, FORWARD, BACKWARD, PORT, STBD, ADJUST, OPEN_CLAW, CLOSE_CLAW, LIFT, SLEEP}

    public abstract void nav_rotate();
    public abstract void nav_to_pos();
    public abstract void claw_open();
    public abstract void claw_close();
    public abstract void claw_raise();
    public abstract void claw_lower();

    // array indexes into a command array
    public static final int OPCODE     = 0;
    public static final int ANGLE      = 1;
    public static final int INCHES     = 1;
    public static final int TARGET     = 1;
    public static final int SECONDS    = 1;
    public static final int POWER      = 2;
    public static final int TOLERANCE  = 3;
    public static final int SURFACE    = 4;

    // execute a sequence of robot commands
    private void execute(Object[][] cmd)
    {
        for (int i=0; cmd != null && i < cmd.length; i++) {
            execute(cmd[i]);
        }
    }

    /*
     * Execute a single command
     */
    private void execute(Object[] cmd)
    {
        // if cmd == null, do nothing because there is no command to execute
        if (cmd == null) return;

        Command op_code = (Command) cmd[OPCODE];
        if (op_code == Command.ROTATE) {
            // rotate the robot, angle is in degrees, positive angle is counterclockwise
            double angle                = (double) cmd[ANGLE];
            double power                = (double) cmd[POWER];
            int    tolerance            = (int)    cmd[TOLERANCE];
            double degrees_to_clicks    = (double) cmd[SURFACE];
            int    clicks               = (int)    (degrees_to_clicks * angle);
            nav_rotate();
        } else if (op_code == Command.FORWARD) {
            // move the robot forward, distance is in inches
            double distance             = (double) cmd[INCHES];
            double power                = (double) cmd[POWER];
            int    tolerance            = (int)    cmd[TOLERANCE];
            double distance_to_clicks   = (double) cmd[SURFACE];
            int    clicks               = (int)    (distance_to_clicks * distance);
            nav_to_pos();
        } else if (op_code == Command.BACKWARD) {
            // move the robot backward, distance is in inches
            double distance             = (double) cmd[INCHES];
            double power                = (double) cmd[POWER];
            int    tolerance            = (int)    cmd[TOLERANCE];
            double distance_to_clicks   = (double) cmd[SURFACE];
            int    clicks               = (int)    (distance_to_clicks * distance);
            nav_to_pos();
        } else if (op_code == Command.PORT) {
            // move the robot laterally to port, distance is in inches
            double distance             = (double) cmd[INCHES];
            double power                = (double) cmd[POWER];
            int    tolerance            = (int)    cmd[TOLERANCE];
            double distance_to_clicks   = (double) cmd[SURFACE];
            int    clicks               = (int)    (distance_to_clicks * distance);
            nav_to_pos();
        } else if (op_code == Command.STBD) {
            // move the robot laterally to starboard, distance is in inches
            double distance             = (double) cmd[INCHES];
            double power                = (double) cmd[POWER];
            int    tolerance            = (int)    cmd[TOLERANCE];
            double distance_to_clicks   = (double) cmd[SURFACE];
            int    clicks               = (int)    (distance_to_clicks * distance);
            nav_to_pos();
        } else if (op_code == Command.ADJUST) {
            // adjust the robot position (laterally, to port or starboard) based on the VuForia VuMark
            // distance is in inches, viewforia_result contains -1 (left), 0 (center), or 1 (right)
            int    vuforia_result = 0;
            double distance             = vuforia_result * (double) cmd[INCHES];
            double power                = (double) cmd[POWER];
            int    tolerance            = (int)    cmd[TOLERANCE];
            double distance_to_clicks   = (double) cmd[SURFACE];
            int    clicks               = (int)    (distance_to_clicks * distance);
            nav_to_pos();
        } else if (op_code == Command.OPEN_CLAW) {
            // open the claw, both sides if there is no qualifier, or just port or starboard if qualifier is included
            if (cmd.length == 1) {
                claw_open();
                claw_close();
            } else if (cmd.length == 2 && (LionAutoInput.Command) cmd[TARGET] == LionAutoInput.Command.PORT) {
                ;
            } else if (cmd.length == 2 && (LionAutoInput.Command) cmd[TARGET] == LionAutoInput.Command.STBD) {
                ;
            }
            sleep(LionConfig.MOTOR_LAG_MILLI);
        } else if (op_code == Command.CLOSE_CLAW) {
            // close the claw, both sides if there is no qualifier, or just port or starboard if qualifier is included
            if (cmd.length == 1) {
                claw_close();
            } else if (cmd.length == 2 && (LionAutoInput.Command) cmd[TARGET] == LionAutoInput.Command.PORT) {
                ;
            } else if (cmd.length == 2 && (LionAutoInput.Command) cmd[TARGET] == LionAutoInput.Command.STBD) {
                ;
            }
            sleep(LionConfig.MOTOR_LAG_MILLI);
        } else if (op_code == Command.LIFT) {
            // raise or lower the claw lift
            int target = (int) cmd[TARGET];
            claw_raise();
        } else if (op_code == Command.SLEEP) {
            // take a rest to wait for motors and servos to catch up
            double seconds = (double) cmd[SECONDS];
            sleep(seconds);
        }
    }

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
