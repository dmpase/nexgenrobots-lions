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
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This file contains ...
 */

public class NgLionDrive extends NgDrive {
    /**************************************************************************
     *                           robot configuration                          *
     **************************************************************************/


    // Expansion Hub Portal 1
    //     Expansion Hub 2
    //         Motors
    public static final String PORT_BOW_NAME        = "front left";             // Hub 2.Motors[0].Rev Robotics HD Hex
    public static final String STBD_BOW_NAME        = "front right";            // Hub 2.Motors[1].Rev Robotics HD Hex
    public static final String STBD_AFT_NAME        = "back right";             // Hub 2.Motors[2].Rev Robotics HD Hex
    public static final String PORT_AFT_NAME        = "back left";              // Hub 2.Motors[3].Rev Robotics HD Hex



    /**************************************************************************
     *                            robot parameters                            *
     **************************************************************************/


    // Misc. motor parameters
    public static final int    MOTOR_TARGET_TOLERANCE =   5;    // encoder clicks
    public static final double MOTOR_LAG_SEC          =   0.25; // seconds
    public static final long   MOTOR_LAG_MILLI        = 250;    // milliseconds


    // Playing field surface constants
    public static final double BALANCING_STONE  = 65;           // encoder clicks per linear inch
    public static final double PLAYING_FIELD    = 75;           // encoder clicks per linear inch
    public static final double ROTATION_RATE    = 10.0;         // encoder clicks per degree of rotation



    /**************************************************************************
     *                            OpMode functions                            *
     **************************************************************************/


    public NgLionDrive(OpMode op_mode)
    {
        super(op_mode);
    }



    /**************************************************************************
     *                             drive functions                            *
     **************************************************************************/


    // locomotion motors
    private DcMotor port_bow_drive = null;
    private DcMotor stbd_bow_drive = null;
    private DcMotor stbd_aft_drive = null;
    private DcMotor port_aft_drive = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()
    {
        if (port_bow_drive == null) {
            port_bow_drive = hardwareMap.get(DcMotor.class, PORT_BOW_NAME);
            port_bow_drive.setDirection(DcMotor.Direction.FORWARD);
            port_bow_drive.setPower(0);
            port_bow_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            port_bow_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (stbd_bow_drive == null) {
            stbd_bow_drive = hardwareMap.get(DcMotor.class, STBD_BOW_NAME);
            stbd_bow_drive.setDirection(DcMotor.Direction.FORWARD);
            stbd_bow_drive.setPower(0);
            stbd_bow_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            stbd_bow_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (stbd_aft_drive == null) {
            stbd_aft_drive = hardwareMap.get(DcMotor.class, STBD_AFT_NAME);
            stbd_aft_drive.setDirection(DcMotor.Direction.FORWARD);
            stbd_aft_drive.setPower(0);
            stbd_aft_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            stbd_aft_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (port_aft_drive == null) {
            port_aft_drive = hardwareMap.get(DcMotor.class, PORT_AFT_NAME);
            port_aft_drive.setDirection(DcMotor.Direction.FORWARD);
            port_aft_drive.setPower(0);
            port_aft_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            port_aft_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    // linear op mode functions
    //    blocking heading change
    //    blocking position change

    // iterative op mode functions
    //    non-blocking heading change
    //    non-blocking position change
    //    wait until change is done


    /*
     * set_new_heading
     *
     * This function turns the robot by the given angle.
     * 1) the heading is specified in degrees
     * 2) the heading is relative to a forward direction (toward the bow)
     * 3) a negative heading turns the robot to port
     * 4) a positive heading turns the robot to starboard
     * 5) power must be between -1 and +1
     * 6) negative power reverses the direction of the turn
     * 7) tolerance specifies +/- the number of encoder clicks
     * 8) this function is suitable for a linear op mode but
     *    may time-out in an iterative op mode
     */

    private static double FIELD_DEGREES_TO_CLICKS = 10.0;
    private static double STONE_DEGREES_TO_CLICKS = 10.0;

    @Override
    public void set_new_heading(double angle, double power, int tolerance, Surface surface)
    {
        double degrees_to_clicks = 0;
        switch (surface) {
        case FIELD:
            degrees_to_clicks = FIELD_DEGREES_TO_CLICKS;
            break;
        case TABLE:
            degrees_to_clicks = STONE_DEGREES_TO_CLICKS;
            break;
        }

        int clicks = (int) (-degrees_to_clicks * angle);

        run_to_position(clicks, clicks, clicks, clicks, power, tolerance);
    }


    /*
     * set_new_position
     *
     * This function moves the robot to a given distance along a given heading.
     * 1) the heading is specified in degrees
     * 2) the heading is relative to a forward direction (toward the bow)
     * 3) a negative heading turns the robot to port
     * 4) a positive heading turns the robot to starboard
     * 5) power must be between -1 and +1
     * 6) negative power reverses the direction of the turn
     * 7) tolerance specifies +/- the number of encoder clicks
     * 8) this function is suitable for a linear op mode but
     *    may time-out in an iterative op mode
     */
    private static double FIELD_INCHES_TO_CLICKS = 75.0;
    private static double STONE_INCHES_TO_CLICKS = 65.0;

    @Override
    public void set_new_position(double heading, double range, double power, int tolerance, Surface surface)
    {
        double angle = heading * Math.PI/180.0 + Math.PI/2.0;
        double inches_to_clicks = 0;
        switch (surface) {
            case FIELD:
                inches_to_clicks = FIELD_INCHES_TO_CLICKS;
                break;
            case TABLE:
                inches_to_clicks = STONE_INCHES_TO_CLICKS;
                break;
        }

        int port_bow_clicks = (int) (- range * Math.sin(angle + Math.PI/4) * inches_to_clicks);
        int stbd_bow_clicks = (int) (  range * Math.cos(angle + Math.PI/4) * inches_to_clicks);
        int stbd_aft_clicks = (int) (  range * Math.sin(angle + Math.PI/4) * inches_to_clicks);
        int port_aft_clicks = (int) (- range * Math.cos(angle + Math.PI/4) * inches_to_clicks);

        run_to_position(port_bow_clicks, stbd_bow_clicks, stbd_aft_clicks, port_aft_clicks, power, tolerance);
    }

    //    non-blocking heading change
    @Override
    public void turn(double power)
    {
        set_drive_power(power, power, power, power);
    }

    //    non-blocking position change
    public static final double POWER_LIMIT = 0.95;
    @Override
    public void move(double bearing, double rotation, double power)
    {
        double port_bow = - power * Math.sin(bearing + Math.PI/4) + rotation;
        double stbd_bow =   power * Math.cos(bearing + Math.PI/4) + rotation;
        double stbd_aft =   power * Math.sin(bearing + Math.PI/4) + rotation;
        double port_aft = - power * Math.cos(bearing + Math.PI/4) + rotation;

        // limit the motors to -1.0 <= motor <= 1.0
        // scale them evenly if adjustments are made
        if (    port_bow < -POWER_LIMIT || POWER_LIMIT < port_bow ||
                stbd_bow < -POWER_LIMIT || POWER_LIMIT < stbd_bow ||
                port_aft < -POWER_LIMIT || POWER_LIMIT < port_aft ||
                stbd_aft < -POWER_LIMIT || POWER_LIMIT < stbd_aft) {

            // find the scale factor
            double max = 0;
            max = (max < Math.abs(port_bow)) ? Math.abs(port_bow) : max;
            max = (max < Math.abs(stbd_bow)) ? Math.abs(stbd_bow) : max;
            max = (max < Math.abs(stbd_aft)) ? Math.abs(stbd_aft) : max;
            max = (max < Math.abs(port_aft)) ? Math.abs(port_aft) : max;

            // scale back the motors evenly
            port_bow = POWER_LIMIT * port_bow / max;
            stbd_bow = POWER_LIMIT * stbd_bow / max;
            stbd_aft = POWER_LIMIT * stbd_aft / max;
            port_aft = POWER_LIMIT * port_aft / max;
        }

        port_bow_drive.setPower(port_bow);
        stbd_bow_drive.setPower(stbd_bow);
        stbd_aft_drive.setPower(stbd_aft);
        port_aft_drive.setPower(port_aft);
    }

    //    set drive motor powers independently
    @Override
    public void set_drive_power(double port_bow, double stbd_bow, double stbd_aft, double port_aft)
    {
        port_bow_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stbd_bow_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stbd_aft_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        port_aft_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        port_bow_drive.setPower(port_bow);
        stbd_bow_drive.setPower(stbd_bow);
        stbd_aft_drive.setPower(stbd_aft);
        port_aft_drive.setPower(port_aft);
    }

    @Override
    public int[] get_drive_encoders()
    {
        int[] cp = new int[4];

        cp[PORT_BOW] = port_bow_drive.getCurrentPosition();
        cp[STBD_BOW] = stbd_bow_drive.getCurrentPosition();
        cp[STBD_AFT] = stbd_aft_drive.getCurrentPosition();
        cp[PORT_AFT] = port_aft_drive.getCurrentPosition();

        return cp;
    }


    /*
     * run_to_position
     *
     * @param fl_tgt
     * @param fr_tgt
     * @param br_tgt
     * @param bl_tgt
     * @param power
     * @param tolerance
     */

    // run the drive motors to a given position,
    // DO reset the encoder before starting (i.e., always start from zero)
    // this routine blocks until the desired location is reached
    private void run_to_position(int fl_tgt, int fr_tgt, int br_tgt, int bl_tgt, double power, int tolerance)
    {
        // cut all power to the motors
        port_bow_drive.setPower(0);
        stbd_bow_drive.setPower(0);
        stbd_aft_drive.setPower(0);
        port_aft_drive.setPower(0);

        // reset the encoders to zero
        port_bow_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stbd_bow_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        port_aft_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stbd_aft_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set the new target position
        port_bow_drive.setTargetPosition(fl_tgt);
        stbd_bow_drive.setTargetPosition(fr_tgt);
        port_aft_drive.setTargetPosition(bl_tgt);
        stbd_aft_drive.setTargetPosition(br_tgt);

        // tell the motors we want them to run to the target position
        port_bow_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stbd_bow_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        port_aft_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stbd_aft_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // apply power to the motors
        port_bow_drive.setPower(power);
        stbd_bow_drive.setPower(power);
        stbd_aft_drive.setPower(power);
        port_aft_drive.setPower(power);

        // wait for all motors to reach their position
        while ( tolerance < Math.abs(port_bow_drive.getTargetPosition() - port_bow_drive.getCurrentPosition()) ||
                tolerance < Math.abs(stbd_bow_drive.getTargetPosition() - stbd_bow_drive.getCurrentPosition()) ||
                tolerance < Math.abs(stbd_aft_drive.getTargetPosition() - stbd_aft_drive.getCurrentPosition()) ||
                tolerance < Math.abs(port_aft_drive.getTargetPosition() - port_aft_drive.getCurrentPosition())) {
            ;
        }

        // sleep while the motors settle into their new position
        sleep(MOTOR_LAG_MILLI);

        // kill all power to the motors
        port_bow_drive.setPower(0);
        stbd_bow_drive.setPower(0);
        stbd_aft_drive.setPower(0);
        port_aft_drive.setPower(0);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {
    }
}
