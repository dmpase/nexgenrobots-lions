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

public class LionRobot extends GriffinRobot {
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

    //         Servos
    public static final String TAIL_NAME            = "tail";                   // Hub 2.Servos[0].Servo
    public static final String BEAM_SWIVEL_NAME     = "beam swivel";            // Hub 2.Servos[3].Servo
    public static final String BEAM_CLAW_NAME       = "beam claw";              // Hub 2.Servos[5].Servo

    //         Analog Input Devices
    public static final String STBD_BOW_IR_NAME     = "starboard bow ir range"; // Hub 2.Analog Input Devices[0].Analog Input
    public static final String STBD_AFT_IR_NAME     = "starboard aft ir range"; // Hub 2.Analog Input Devices[1].Analog Input
    public static final String PORT_BOW_IR_NAME     = "port bow ir range";      // Hub 2.Analog Input Devices[2].Analog Input
    public static final String PORT_AFT_IR_NAME     = "port aft ir range";      // Hub 2.Analog Input Devices[3].Analog Input

    //         I2C Bus 0
    public static final String IMU0_NAME            = "imu 0";                  // Hub 2.I2C Bus 0[0].Rev Expansion Hub IMU

    //         I2C Bus 1
    public static final String STBD_BOW_MR_RANGE_NAME = "stbd bow mr range";     // Hub 2.I2C Bus 1[0].MR Range Sensor
    public static final String STBD_AFT_MR_RANGE_NAME = "stbd aft mr range";     // Hub 2.I2C Bus 1[0].MR Range Sensor

    //         I2C Bus 2
    public static final String PORT_BOW_MR_RANGE_NAME = "port bow mr range";     // Hub 2.I2C Bus 2[0].MR Range Sensor
    public static final String PORT_AFT_MR_RANGE_NAME = "port bow mr range";     // Hub 2.I2C Bus 2[0].MR Range Sensor

    //         I2C Bus 3
    public static final String REV_COLOR_RANGE_NAME = "rev color range";        // Hub 2.I2C Bus 3[0].Rev Color/Range Sensor


    //     Expansion Hub 3
    //         Motors
    public static final String BEAM_DRIVE_NAME      = "beam drive";             // Hub 3.Motors[2].Rev Robotics HD Hex
    public static final String LIFT_DRIVE_NAME      = "claw lift";              // Hub 3.Motors[3].Rev Robotics HD Hex

    //         Servos
    public static final String PORT_CLAW_NAME       = "port claw";              // Hub 3.Servos[0].Servo
    public static final String STBD_CLAW_NAME       = "starboard claw";         // Hub 3.Servos[5].Servo

    //         I2C Bus 0
    public static final String IMU1_NAME            = "imu 1";                  // Hub 3.I2C Bus 0[0].Rev Expansion Hub IMU



    /**************************************************************************
     *                            robot parameters                            *
     **************************************************************************/


    // VuForia License Key
    public static final String VUFORIA_LICENSE_KEY = "AWVXYZn/////AAAAGcG6g8XSSUMJsDaizcApOtsaA0fWzUQwImrdEn1MqH4JNqCzUwlyvEX0YALy7XyUeSpiANJkBg9kplUtcniUZKw8bF0dSpEfXZKXxn1yhbIohmpVmIK+Ngv1imYrkY6ePmvTfO2IpyQi5yO5ZmfSC8OzlH+XEMD0vRIXHMhxFpin7vTIHaoz8MEifSjRTznh1ZUSRnJfQ01KvMHEefES0kwhehlEKoqgpNMOYg0B5pV0bDDi9/Qh4eMR7sEk1GSx3QPxl/lYuZVcWSh8DutXv8oo9LhnbAaHTecCAR6gnNODow0WUAH2N9vxdLOjk2UfWVEJgqmHembIDHRzJN4fjcOECTFfLHIVmZ66GwgjPWxV";
    public static final VuforiaLocalizer.CameraDirection CAMERA_DIRECTION = VuforiaLocalizer.CameraDirection.FRONT;


    // Lift parameters
    public static final double LIFT_POWER       = 0.75;
    public static final int    LIFT_TARGET_HI   = 5000;
    public static final int    LIFT_TARGET_INCH =  400;
    public static final int    LIFT_TARGET_LO   =  200;
    public static final int    LIFT_TARGET_SET  =  400;
    public static final DcMotor.Direction LIFT_DIRECTION = DcMotor.Direction.FORWARD;


    // Claw parameters
    public static final double PORT_CLAW_OPENED = 0.90;
    public static final double PORT_CLAW_CLOSED = 0.25;
    public static final double STBD_CLAW_OPENED = 0.00;
    public static final double STBD_CLAW_CLOSED = 0.65;


    // Tail parameters
    public static final double TAIL_POS_UP      = 0.00;
    public static final double TAIL_POS_DN      = 0.90;


    // Beam parameters
    public static final double BEAM_POWER       = 0.40;
    public static final int    BEAM_TARGET_IN   =    0;         // encoder clicks for full retraction
    public static final int    BEAM_TARGET_OUT  = -15000;       // encoder clicks for full extension
    public static final DcMotor.Direction BEAM_DIRECTION = DcMotor.Direction.FORWARD;
    public static final double BEAM_CLAW_OPENED = 0.00;
    public static final double BEAM_CLAW_CLOSED = 0.90;
    public static final double BEAM_SWIVEL_UP   = 0.50;
    public static final double BEAM_SWIVEL_DOWN = 0.00;

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


    // hardware map and telemetry from the OpMode class
    public HardwareMap hardwareMap = null;
    public Telemetry telemetry = null;

    public LionRobot(OpMode op_mode)
    {
        super(op_mode);

        hardwareMap = op_mode.hardwareMap;
        telemetry = op_mode.telemetry;
    }


    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()
    {
        telemetry.addData("Status", "Starting Initialization.");

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialization Complete.");
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start()
    {
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {
    }



    /**************************************************************************
     *                             drive functions                            *
     **************************************************************************/


    // locomotion motors
    private DcMotor port_bow_drive = null;
    private DcMotor stbd_bow_drive = null;
    private DcMotor stbd_aft_drive = null;
    private DcMotor port_aft_drive = null;

    @Override
    public void drive_init()
    {
        if (port_bow_drive == null) {
            port_bow_drive = hardwareMap.get(DcMotor.class, EagleConfig.PORT_BOW);
            port_bow_drive.setDirection(DcMotor.Direction.FORWARD);
            port_bow_drive.setPower(0);
            port_bow_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            port_bow_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (stbd_bow_drive == null) {
            stbd_bow_drive = hardwareMap.get(DcMotor.class, EagleConfig.STBD_BOW);
            stbd_bow_drive.setDirection(DcMotor.Direction.FORWARD);
            stbd_bow_drive.setPower(0);
            stbd_bow_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            stbd_bow_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (stbd_aft_drive == null) {
            stbd_aft_drive = hardwareMap.get(DcMotor.class, EagleConfig.STBD_AFT);
            stbd_aft_drive.setDirection(DcMotor.Direction.FORWARD);
            stbd_aft_drive.setPower(0);
            stbd_aft_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            stbd_aft_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (port_aft_drive == null) {
            port_aft_drive = hardwareMap.get(DcMotor.class, EagleConfig.PORT_AFT);
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



    /**************************************************************************
     *                             claw functions                              *
     **************************************************************************/


    // claw and tail servos
    private Servo port_claw = null;
    private Servo stbd_claw = null;

    // claw lift motor
    private DcMotor lift    = null;

    // initialize the claw
    @Override
    public void claw_init()
    {
        if (port_claw == null) {
            port_claw = hardwareMap.get(Servo.class, PORT_CLAW_NAME);
            port_claw.setDirection(Servo.Direction.FORWARD);
        }

        if (stbd_claw == null) {
            stbd_claw = hardwareMap.get(Servo.class, STBD_CLAW_NAME);
            stbd_claw.setDirection(Servo.Direction.FORWARD);
        }

        if (lift == null) {
            lift = hardwareMap.get(DcMotor.class, LIFT_DRIVE_NAME);
            lift.setDirection(LIFT_DIRECTION);
            lift.setPower(0);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    @Override
    public void claw_open()
    {
        port_claw.setPosition(PORT_CLAW_OPENED);
        stbd_claw.setPosition(STBD_CLAW_OPENED);
    }

    @Override
    public void claw_close()
    {
        port_claw.setPosition(PORT_CLAW_CLOSED);
        stbd_claw.setPosition(STBD_CLAW_CLOSED);
    }

    @Override
    public void claw_raise(double height)
    {
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int target = (int) (LIFT_TARGET_LO + height * (LIFT_TARGET_HI - LIFT_TARGET_LO));
        lift.setTargetPosition(target);
        lift.setPower(LIFT_POWER);
    }

    @Override
    public void claw_stop()
    {
        lift.setPower(0);
    }

    @Override
    public void claw_wait()
    {
        double start = runtime.seconds();
        while ( MOTOR_TARGET_TOLERANCE < Math.abs(lift.getTargetPosition() - lift.getCurrentPosition()) ) {
            if (MOTOR_LAG_SEC < (runtime.seconds() - start)) break;
        }

        if ((runtime.seconds() - start) < MOTOR_LAG_SEC) {
            sleep(MOTOR_LAG_SEC - (start - runtime.seconds()));
        }

        lift.setPower(0);
    }



    /**************************************************************************
     *                              tail functions                            *
     **************************************************************************/


    // claw and tail servos
    private Servo tail = null;

    // initialize the tail
    @Override
    public void tail_init()
    {
        if (tail == null) {
            tail = hardwareMap.get(Servo.class, TAIL_NAME);
            tail.setDirection(Servo.Direction.FORWARD);
        }
    }

    @Override
    public void tail_raise(double height)
    {
        double pos = TAIL_POS_DN + height * (TAIL_POS_UP - TAIL_POS_DN);
        tail.setPosition(height);
    }



    /**************************************************************************
     *                              beam functions                            *
     **************************************************************************/


    // beam motor
    private DcMotor beam_drive  = null;

    // beam servos
    private Servo beam_claw     = null;
    private Servo beam_swivel   = null;

    // initialize the beam
    @Override
    public void beam_init()
    {
        if (beam_claw == null) {
            beam_claw = hardwareMap.get(Servo.class, BEAM_CLAW_NAME);
            beam_claw.setDirection(Servo.Direction.FORWARD);
        }

        if (beam_swivel == null) {
            beam_swivel = hardwareMap.get(Servo.class, BEAM_SWIVEL_NAME);
            beam_swivel.setDirection(Servo.Direction.FORWARD);
        }

        if (beam_drive == null) {
            beam_drive = hardwareMap.get(DcMotor.class, BEAM_DRIVE_NAME);
            beam_drive.setDirection(DcMotor.Direction.FORWARD);
            beam_drive.setPower(0);
            beam_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            beam_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    @Override
    public void beam_open()
    {
        beam_claw.setPosition(BEAM_CLAW_OPENED);
    }

    @Override
    public void beam_close()
    {
        beam_claw.setPosition(BEAM_CLAW_CLOSED);
    }

    @Override
    public void beam_raise()
    {
        beam_swivel.setPosition(BEAM_SWIVEL_UP);
    }

    @Override
    public void beam_lower()
    {
        beam_swivel.setPosition(BEAM_SWIVEL_DOWN);
    }

    @Override
    public void beam_extend(double length)
    {
        beam_drive.setPower(0);
        int target = (int) (BEAM_TARGET_IN + length * (BEAM_TARGET_OUT - BEAM_TARGET_IN));
        beam_drive.setTargetPosition(target);
        beam_drive.setPower(BEAM_POWER);
    }

    @Override
    public void beam_stop()
    {
        beam_drive.setPower(0);
    }

    @Override
    public void beam_wait()
    {
        double start = runtime.seconds();
        while ( MOTOR_TARGET_TOLERANCE < Math.abs(beam_drive.getTargetPosition() - beam_drive.getCurrentPosition()) ) {
            if (MOTOR_LAG_SEC < (runtime.seconds() - start)) break;
        }

        if ((runtime.seconds() - start) < MOTOR_LAG_SEC) {
            sleep(MOTOR_LAG_SEC - (start - runtime.seconds()));
        }
    }



    /**************************************************************************
     *                           vuforia functions                            *
     **************************************************************************/


    // VuForia objects
    int cameraMonitorViewId = -1;
    VuforiaLocalizer vuforia = null;
    VuforiaLocalizer.Parameters vuforia_parameters = null;
    VuforiaTrackables relicTrackables = null;
    VuforiaTrackable relicTemplate = null;
    RelicRecoveryVuMark vuMark = null;

    // initialize the vuforia mechanism
    @Override
    public void vuforia_init()
    {
        telemetry.addData("Status", "Initializing VuForia.");

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        vuforia_parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vuforia_parameters.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforia_parameters.cameraDirection = CAMERA_DIRECTION;
        vuforia = ClassFactory.createVuforiaLocalizer(vuforia_parameters);
        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        relicTrackables.activate();
    }

    @Override
    public int vuforia_read()
    {
        int vuforia_result = VUFORIA_CENTER;

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        String vumark_position = vuMark.name();
        if (vumark_position.equalsIgnoreCase("LEFT")) {
            vuforia_result = VUFORIA_LEFT;
        } else if (vumark_position.equalsIgnoreCase("CENTER")) {
            vuforia_result = VUFORIA_CENTER;
        } else if (vumark_position.equalsIgnoreCase("RIGHT")) {
            vuforia_result = VUFORIA_RIGHT;
        }

        return vuforia_result;
    }



    /**************************************************************************
     *                            sensor functions                            *
     **************************************************************************/


    // Modern Robotics ultrasonic range sensors
    private DistanceSensor port_bow_mr_range = null;
    private DistanceSensor stbd_bow_mr_range = null;
    private DistanceSensor stbd_aft_mr_range = null;
    private DistanceSensor port_aft_mr_range = null;

    // REV Robotics distance/color sensor
    private ColorSensor color_sensor = null;
    private DistanceSensor distance_sensor = null;

    // Pololu IR range sensors
    AnalogInput port_aft_ir = null;
    AnalogInput stbd_aft_ir = null;
    AnalogInput port_bow_ir = null;
    AnalogInput stbd_bow_ir = null;
    Distance    ir_v2in     = new Distance(10.616758844230123, -2.625694922444332, 5.292315651154265);

    @Override
    public void sensor_init()
    {
        port_bow_mr_range = hardwareMap.get(DistanceSensor.class, PORT_BOW_MR_RANGE_NAME);
        stbd_bow_mr_range = hardwareMap.get(DistanceSensor.class, STBD_BOW_MR_RANGE_NAME);
        stbd_aft_mr_range = hardwareMap.get(DistanceSensor.class, STBD_AFT_MR_RANGE_NAME);
        port_aft_mr_range = hardwareMap.get(DistanceSensor.class, PORT_AFT_MR_RANGE_NAME);

        color_sensor    = hardwareMap.get(ColorSensor.class,    REV_COLOR_RANGE_NAME);
        distance_sensor = hardwareMap.get(DistanceSensor.class, REV_COLOR_RANGE_NAME);

        port_bow_ir = hardwareMap.get(AnalogInput.class, PORT_BOW_IR_NAME);
        stbd_bow_ir = hardwareMap.get(AnalogInput.class, STBD_BOW_IR_NAME);
        stbd_aft_ir = hardwareMap.get(AnalogInput.class, STBD_AFT_IR_NAME);
        port_aft_ir = hardwareMap.get(AnalogInput.class, PORT_AFT_IR_NAME);
    }

    @Override
    public double[] sensor_color()
    {
        double[] rv = new double[4];

        rv[ALPHA] = color_sensor.alpha();
        rv[RED  ] = color_sensor.red();
        rv[GREEN] = color_sensor.green();
        rv[BLUE ] = color_sensor.blue();

        return rv;
    }

    @Override
    public double[] sensor_down()
    {
        double[] rv = new double[4];

        rv[PORT_BOW] = ir_v2in.distance(port_bow_ir.getVoltage());
        rv[STBD_BOW] = ir_v2in.distance(stbd_bow_ir.getVoltage());
        rv[STBD_AFT] = ir_v2in.distance(stbd_aft_ir.getVoltage());
        rv[PORT_AFT] = ir_v2in.distance(port_aft_ir.getVoltage());

        return rv;
    }

    @Override
    public double[] sensor_out()
    {
        double[] rv = new double[4];

        rv[PORT_BOW] = port_bow_mr_range.getDistance(DistanceUnit.INCH);
        rv[STBD_BOW] = stbd_bow_mr_range.getDistance(DistanceUnit.INCH);
        rv[STBD_AFT] = stbd_aft_mr_range.getDistance(DistanceUnit.INCH);
        rv[PORT_AFT] = port_aft_mr_range.getDistance(DistanceUnit.INCH);

        return rv;
    }
}
