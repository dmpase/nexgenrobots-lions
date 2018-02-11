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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This file contains ...
 */

public class LionRobot extends GriffinRobot {
    // hardware map and telemetry from the OpMode class
    public HardwareMap hardwareMap = null;
    public Telemetry telemetry = null;

    public LionRobot(OpMode op_mode)
    {
        super(op_mode);

        hardwareMap = op_mode.hardwareMap;
        telemetry = op_mode.telemetry;
    }

    // locomotion motors
    private DcMotor port_bow_drive = null;
    private DcMotor stbd_bow_drive = null;
    private DcMotor stbd_aft_drive = null;
    private DcMotor port_aft_drive = null;

    // claw and tail servos
    private Servo claw = null;
    private Servo tail = null;

    // claw lift and beam motors
    private DcMotor lift        = null;
    private DcMotor beam_drive  = null;

    // beam servos
    private Servo beam_claw     = null;
    private Servo beam_swivel   = null;

    // Modern Robotics ultrasonic range sensors
    private DistanceSensor port_mr_range = null;
    private DistanceSensor stbd_mr_range = null;

    // REV Robotics distance/color sensor
    private ColorSensor color_sensor = null;
    private DistanceSensor distance_sensor = null;

    // Pololu IR range sensors
    AnalogInput port_ir_aft = null;
    AnalogInput stbd_ir_aft = null;
    AnalogInput port_ir_bow = null;
    AnalogInput stbd_ir_bow = null;
    Distance    ir_v2in     = new Distance(10.616758844230123, -2.625694922444332, 5.292315651154265);

    // VuForia objects
    int cameraMonitorViewId = -1;
    VuforiaLocalizer vuforia = null;
    VuforiaLocalizer.Parameters vuforia_parameters = null;
    VuforiaTrackables relicTrackables = null;
    VuforiaTrackable relicTemplate = null;
    RelicRecoveryVuMark vuMark = null;


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


    private static double FIELD_DEGREES_TO_CLICKS = 10.0;
    private static double STONE_DEGREES_TO_CLICKS = 10.0;

    @Override
    public void nav_rotate(double angle, double power, int tolerance, Surface surface)
    {
        int clicks= 0;
        if (surface == Surface.FIELD) {
            clicks = (int) (FIELD_DEGREES_TO_CLICKS * angle);
        } else if (surface == Surface.TABLE) {
            clicks = (int) (STONE_DEGREES_TO_CLICKS * angle);
        }

        run_to_position(clicks, clicks, clicks, clicks, power, tolerance);
    }


    private static double FIELD_INCHES_TO_CLICKS = 75.0;
    private static double STONE_INCHES_TO_CLICKS = 65.0;

    @Override
    public void nav_to_pos(double heading, double range, double power, int tolerance, Surface surface)
    {
        double angle = heading * Math.PI/180.0 + Math.PI/2.0;
        double inches_to_clicks = 0;
        if (surface == Surface.FIELD) {
            inches_to_clicks = FIELD_INCHES_TO_CLICKS;
        } else if (surface == Surface.TABLE) {
            inches_to_clicks = STONE_INCHES_TO_CLICKS;
        }

        int port_bow_clicks = (int) (- range * Math.sin(angle + Math.PI/4) * inches_to_clicks);
        int stbd_bow_clicks = (int) (  range * Math.cos(angle + Math.PI/4) * inches_to_clicks);
        int stbd_aft_clicks = (int) (  range * Math.sin(angle + Math.PI/4) * inches_to_clicks);
        int port_aft_clicks = (int) (- range * Math.cos(angle + Math.PI/4) * inches_to_clicks);

        run_to_position(port_bow_clicks, stbd_bow_clicks, stbd_aft_clicks, port_aft_clicks, power, tolerance);
    }

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
    public void claw_open()
    {
    }

    @Override
    public void claw_close()
    {
    }

    @Override
    public void claw_raise()
    {
    }

    @Override
    public void claw_lower()
    {
    }

    @Override
    public void beam_open()
    {
    }

    @Override
    public void beam_close()
    {
    }

    @Override
    public void beam_raise()
    {
    }

    @Override
    public void beam_lower()
    {
    }

    @Override
    public void beam_extend()
    {
    }

    @Override
    public void beam_retract()
    {
    }

    @Override
    public void vuforia_read()
    {
    }

    @Override
    public void vuforia_adjust(double distance, double power, int tolerance, Surface surface)
    {
    }


    /**************************************************************************
     *               private support functions for this robot                 *
     **************************************************************************/


    private void init_drive()
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
        sleep(LionConfig.MOTOR_LAG_MILLI);

        // kill all power to the motors
        port_bow_drive.setPower(0);
        stbd_bow_drive.setPower(0);
        stbd_aft_drive.setPower(0);
        port_aft_drive.setPower(0);
    }
}
