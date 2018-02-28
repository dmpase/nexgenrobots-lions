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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This file contains an abstract definition of a minimal robot - no devices or hardware.
 * Inherit this class for your robot definition and provide init(), start(), and stop()
 * functions to manage any additional devices your robot requires.
 */

public abstract class GriffinRobot {
    // hardware map and telemetry from the OpMode class
    public HardwareMap hardwareMap = null;
    public Telemetry   telemetry   = null;

    public enum OpModeType { Unknown, Linear, Iterative }
    public OpModeType op_mode_type = OpModeType.Unknown;

    public GriffinRobot(OpMode op_mode)
    {
        hardwareMap = op_mode.hardwareMap;
        telemetry   = op_mode.telemetry;

        for (Object next = op_mode; 0 != next.getClass().getSimpleName().compareToIgnoreCase("Object"); next = next.getClass().getSuperclass()) {
            if (0 == next.getClass().getSimpleName().compareToIgnoreCase("LinearOpMode")) {
                op_mode_type = OpModeType.Linear;
                break;
            } else if (0 == next.getClass().getSimpleName().compareToIgnoreCase("OpMode")) {
                op_mode_type = OpModeType.Iterative;
                break;
            }
        }
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


    public static enum Command {DRIVE, CLAW, BEAM, VUFORIA, SLEEP}
    public static enum SubCmd  {NONE, TURN, LOCATION, POWER, OPEN, CLOSE, RAISE, LOWER, EXTEND, RETRACT, READ, ADJUST}
    public static enum Surface {FIELD, TABLE}


    // initialize drive motors
    public abstract void drive_init();

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
    public static final int PORT_BOW = 0;
    public static final int STBD_BOW = 1;
    public static final int STBD_AFT = 2;
    public static final int PORT_AFT = 3;
    public abstract int[] get_drive_encoders();


    // initialize claw motors
    public static final double CLAW_MIN_HEIGHT = 0.0;
    public static final double CLAW_MAX_HEIGHT = 1.0;
    public abstract void claw_init();
    public abstract void claw_open();
    public abstract void claw_close();
    public abstract void claw_raise(double height);
    public abstract void claw_stop();
    public abstract void claw_wait();

    // initialize tail motors
    public abstract void tail_init();
    public abstract void tail_raise(double height);

    // initialize beam motors
    public static final double BEAM_MIN_LENGTH = 0.0;
    public static final double BEAM_MAX_LENGTH = 1.0;
    public abstract void beam_init();
    public abstract void beam_open();
    public abstract void beam_close();
    public abstract void beam_raise();
    public abstract void beam_lower();
    public abstract void beam_extend(double length);
    public abstract void beam_stop();
    public abstract void beam_wait();

    public abstract void vuforia_init();
    public abstract int  vuforia_read();

    // array indexes into a command array
    public static final int OPCODE     = 0;
    public static final int SUBCODE    = 1;
    public static final int ARG0       = 2;
    public static final int ARG1       = 3;
    public static final int ARG2       = 4;
    public static final int ARG3       = 5;
    public static final int ARG4       = 6;

    public static final double FORWARD   =   0;
    public static final double STBD      =  90;
    public static final double STARBOARD =  90;
    public static final double AFT       = 180;
    public static final double BACKWARD  = 180;
    public static final double PORT      = 270;

    // position indicators for LEFT, CENTER and RIGHT vumark images
    public static final int VUFORIA_LEFT   = -1;
    public static final int VUFORIA_CENTER =  0;
    public static final int VUFORIA_RIGHT  =  1;
    public int vuforia_result = VUFORIA_CENTER;

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
        if (cmd == null || cmd.length < 2) return;

        Command op_code = (Command) cmd[OPCODE];
        SubCmd  sub_cmd = (SubCmd ) cmd[SUBCODE];
        if (op_code == Command.DRIVE) {
            if (sub_cmd == SubCmd.TURN) {
                // rotate the robot, angle is in degrees, positive angle is counterclockwise
                // +90 degrees is to starboard, 180 degrees is aft, -90 or +270 degrees is to port
                // DRIVE, TURN, angle, power, tolerance, surface
                double  angle     = (double)  cmd[ARG0];
                double  power     = (double)  cmd[ARG1];
                int     tolerance = (int)     cmd[ARG2];
                Surface surface   = (Surface) cmd[ARG3];
                set_new_heading(angle, power, tolerance, surface);
            } else if (sub_cmd == SubCmd.LOCATION) {
                // move the robot forward, backward or laterally, distance is in inches
                // DRIVE, LOCATION, heading, range, power, tolerance, surface
                double  heading   = (double)  cmd[ARG0];    // bearing means angle in degrees
                double  range     = (double)  cmd[ARG1];    // range means distance in inches
                double  power     = (double)  cmd[ARG2];    // power settings for the drive motors
                int     tolerance = (int)     cmd[ARG3];    // tolerance in encoder clicks
                Surface surface   = (Surface) cmd[ARG4];    // type of surface, e.g., playing field
                set_new_position(heading, range, power, tolerance, surface);
            } else if (sub_cmd == SubCmd.POWER) {
                // set the power on the drive motors
                // +1.0 is full forward (clockwise), -1.0 is full reverse (counterclockwise)
                double port_bow = (double) cmd[ARG0];
                double stbd_bow = (double) cmd[ARG1];
                double stbd_aft = (double) cmd[ARG2];
                double port_aft = (double) cmd[ARG3];
                set_drive_power(port_bow, stbd_bow, stbd_aft, port_aft);
            }
        } else if (op_code == Command.CLAW) {
            if (sub_cmd == SubCmd.OPEN) {
                claw_open();
            } else if (sub_cmd == SubCmd.CLOSE) {
                claw_close();
            } else if (sub_cmd == SubCmd.RAISE) {
                claw_raise(CLAW_MAX_HEIGHT);
            } else if (sub_cmd == SubCmd.LOWER) {
                claw_raise(CLAW_MIN_HEIGHT);
            }
        } else if (op_code == Command.BEAM) {
            if (sub_cmd == SubCmd.OPEN) {
                beam_open();
            } else if (sub_cmd == SubCmd.CLOSE) {
                beam_close();
            } else if (sub_cmd == SubCmd.RAISE) {
                beam_raise();
            } else if (sub_cmd == SubCmd.LOWER) {
                beam_lower();
            } else if (sub_cmd == SubCmd.EXTEND) {
                beam_extend(BEAM_MAX_LENGTH);
            } else if (sub_cmd == SubCmd.RETRACT) {
                beam_extend(BEAM_MIN_LENGTH);
            }
        } else if (op_code == Command.VUFORIA) {
            // adjust the robot position (laterally, to port or starboard) based on the VuForia VuMark
            // distance is in inches, viewforia_result contains -1 (left), 0 (center), or 1 (right)
            if (sub_cmd == SubCmd.READ) {
                vuforia_result = vuforia_read();
            } else if (sub_cmd == SubCmd.ADJUST) {
                double  bearing   = vuforia_result * STARBOARD;
                double  range     = (double)  cmd[ARG1];    // range means distance in inches
                double  power     = (double)  cmd[ARG2];    // power settings for the drive motors
                int     tolerance = (int)     cmd[ARG3];    // tolerance in encoder clicks
                Surface surface   = (Surface) cmd[ARG4];    // type of surface, e.g., playing field
                set_new_position(bearing, range, power, tolerance, surface);
            }
        } else if (op_code == Command.SLEEP) {
            // take a rest to wait for motors and servos to catch up
            double seconds = (double) cmd[ARG0];
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
