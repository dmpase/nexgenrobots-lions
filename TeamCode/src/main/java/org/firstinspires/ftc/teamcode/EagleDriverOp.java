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
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Eagle Driver OpMode", group="Iterative OpMode")
// @Disabled
public class EagleDriverOp extends OpMode
{
    // Declare OpMode members.
    // REV Robotics drive motors
    private DcMotor port_bow_drive = null;
    private DcMotor stbd_bow_drive = null;
    private DcMotor stbd_aft_drive = null;
    private DcMotor port_aft_drive = null;

    // claw and tail servos
    private Servo claw             = null;

    // claw lift and beam drive motors
    private DcMotor lift           = null;
    private DcMotor beam_drive = null;

    // beam servos
    private Servo beam_claw        = null;
    private Servo beam_swivel      = null;

    private ElapsedTime runtime    = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()
    {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        telemetry.addData("Status", "Initializing Motors.");
        init_drive();

        telemetry.addData("Status", "Initializing Servos.");
        init_claw();

        telemetry.addData("Status", "Initializing Beam.");
        init_beam();

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialization Complete.");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {
        get_motor_settings();

        get_claw_settings();

        get_beam_settings();

        // Show the elapsed game time and other data.
        telemetry.addData("Beam", "%5d", beam_drive.getCurrentPosition());
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }


    public void get_beam_settings()
    {
        if ( gamepad2.dpad_left && ! gamepad2.dpad_right ) {            // extend the beam
            beam_drive.setPower(0);
            double start = runtime.seconds();
            beam_drive.setTargetPosition(EagleConfig.BEAM_TARGET_OUT);
            beam_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            beam_drive.setPower(EagleConfig.BEAM_POWER);
        } else if ( ! gamepad2.dpad_left && gamepad2.dpad_right ) {     // retract the beam
            beam_drive.setPower(0);
            double start = runtime.seconds();
            beam_drive.setTargetPosition(EagleConfig.BEAM_TARGET_IN);
            beam_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            beam_drive.setPower(-EagleConfig.BEAM_POWER);
        } else {
            beam_drive.setPower(0);
        }

        if ( gamepad2.dpad_up && ! gamepad2.dpad_down )     {    // swivel beam claw up
            beam_swivel.setPosition(EagleConfig.BEAM_SWIVEL_UP);
        } else if ( ! gamepad2.dpad_up && gamepad2.dpad_down )     {    // swivel beam claw down
            beam_swivel.setPosition(EagleConfig.BEAM_SWIVEL_DOWN);
        } else if ( ! gamepad2.left_bumper && gamepad2.right_bumper ) { // open beam claw
            beam_claw.setPosition(EagleConfig.BEAM_CLAW_OPENED);
        } else if ( gamepad2.left_bumper && ! gamepad2.right_bumper ) { // close beam claw
            beam_claw.setPosition(EagleConfig.BEAM_CLAW_CLOSED);
        }
    }


    public static final double lift_max_pwr =  0.40;

    public void get_claw_settings()
    {23
        if (! gamepad2.x && gamepad2.b && ! gamepad1.start && ! gamepad2.start) {           // open the claw
            claw.setPosition(EagleConfig.CLAW_OPENED);
        } else if (gamepad2.x && ! gamepad2.b && ! gamepad1.start && ! gamepad2.start) {    // close the claw
            claw.setPosition(EagleConfig.CLAW_CLOSED);
        }

        if (gamepad2.y && ! gamepad2.a && ! gamepad1.start && ! gamepad2.start) {           // raise the claw
            lift.setPower(-EagleConfig.LIFT_POWER);
        } else if (! gamepad2.y && gamepad2.a && ! gamepad1.start && ! gamepad2.start) {    // lower the claw
            lift.setPower(EagleConfig.LIFT_POWER);
        } else {
            lift.setPower(0);
        }
    }


    public static final double POWER_LIMIT = 0.95;

    public static final double stick_dead_zone   = 0.05;
    public static final double full_speed        = 0.40;
    public static final double half_speed        = 0.20;

    public static final double trigger_dead_zone = 0.05;
    public static final double half_turn         = 0.20;
    public static final double slow_turn         = 0.10;

    public static final int PORT_BOW             = 0;
    public static final int STBD_BOW             = 1;
    public static final int STBD_AFT             = 2;
    public static final int PORT_AFT             = 3;
    public static final int MOTOR_COUNT          = 4;

    public void get_motor_settings()
    {
        // use game pad 1 right stick to determinie speed and bearing UNLESS the left stick is being used
        // right stick is for speed, left stick is for precision
        double x = (-stick_dead_zone < gamepad1.left_stick_x && gamepad1.left_stick_x < stick_dead_zone) ? 0 : full_speed * gamepad1.left_stick_x;
        double y = (-stick_dead_zone < gamepad1.left_stick_y && gamepad1.left_stick_y < stick_dead_zone) ? 0 : full_speed * gamepad1.left_stick_y;
        if (gamepad1.right_stick_x < -stick_dead_zone || stick_dead_zone < gamepad1.right_stick_x ||
            gamepad1.right_stick_y < -stick_dead_zone || stick_dead_zone < gamepad1.right_stick_y) {

            x = half_speed * gamepad1.right_stick_x;
            y = half_speed * gamepad1.right_stick_y;
        }

        // get the angle of the stick to compute the desired bearing
        double alpha   = (x == 0 && y == 0) ? 0 : Math.atan2(y, x);
        double bearing = (x == 0 && y == 0) ? 0 : alpha + Math.PI/2.0;

        // limit the throttle
        double throttle = Math.sqrt(x*x + y*y);
        throttle = (POWER_LIMIT < throttle) ? POWER_LIMIT : throttle;

        // add in rotation, if any
        double rotation = 0;
        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            // conflicting inputs, do nothing
        } else if (gamepad1.left_bumper) {
            // bumpers take priority, turn left
            rotation = slow_turn;
        } else if (gamepad1.right_bumper) {
            // bumpers take priority, turn right
            rotation = -slow_turn;
        } else if (trigger_dead_zone < gamepad1.left_trigger && trigger_dead_zone < gamepad1.right_trigger) {
            // conflicting inputs, do nothing
        } else if (trigger_dead_zone < gamepad1.left_trigger) {
            // turn left using trigger
            rotation = half_turn * gamepad1.left_trigger;
        } else if (trigger_dead_zone < gamepad1.right_trigger) {
            // turn right using trigger
            rotation = -half_turn * gamepad1.right_trigger;
        }

        double[] motors = new double[MOTOR_COUNT];

        motors[PORT_BOW] = - throttle * Math.sin(bearing + Math.PI/4) + rotation;
        motors[STBD_BOW] =   throttle * Math.cos(bearing + Math.PI/4) + rotation;
        motors[STBD_AFT] =   throttle * Math.sin(bearing + Math.PI/4) + rotation;
        motors[PORT_AFT] = - throttle * Math.cos(bearing + Math.PI/4) + rotation;

        // limit the motors to -1.0 <= motor <= 1.0
        // scale them evenly if adjustments are made
        if (motors[PORT_BOW] < -POWER_LIMIT || POWER_LIMIT < motors[PORT_BOW] ||
            motors[STBD_BOW] < -POWER_LIMIT || POWER_LIMIT < motors[STBD_BOW] ||
            motors[PORT_AFT] < -POWER_LIMIT || POWER_LIMIT < motors[PORT_AFT] ||
            motors[STBD_AFT] < -POWER_LIMIT || POWER_LIMIT < motors[STBD_AFT]) {

            // find the scale factor
            double max = 0;
            for (int i=0; i < motors.length; i++) {
                double abs = Math.abs(motors[i]);
                max = (max < abs) ? abs : max;
            }

            // scale back the motors evenly
            for (int i=0; i < motors.length; i++) {
                motors[i] = POWER_LIMIT * motors[i] / max;
            }
        }

        port_bow_drive.setPower(motors[PORT_BOW]);
        stbd_bow_drive.setPower(motors[STBD_BOW]);
        stbd_aft_drive.setPower(motors[STBD_AFT]);
        port_aft_drive.setPower(motors[PORT_AFT]);
    }

    private void init_drive() {
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

    public void init_claw()
    {
        if (claw == null) {
            claw = hardwareMap.get(Servo.class, EagleConfig.CLAW);
            claw.setDirection(Servo.Direction.FORWARD);
        }

        if (lift == null) {
            lift = hardwareMap.get(DcMotor.class, EagleConfig.LIFT_DRIVE);
            lift.setDirection(EagleConfig.LIFT_DIRECTION);
            lift.setPower(0);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void init_beam()
    {
        if (beam_claw == null) {
            beam_claw = hardwareMap.get(Servo.class, EagleConfig.BEAM_CLAW);
            beam_claw.setDirection(Servo.Direction.FORWARD);
        }

        if (beam_swivel == null) {
            beam_swivel = hardwareMap.get(Servo.class, EagleConfig.BEAM_SWIVEL);
            beam_swivel.setDirection(Servo.Direction.FORWARD);
        }

        if (beam_drive == null) {
            beam_drive = hardwareMap.get(DcMotor.class, EagleConfig.BEAM_DRIVE);
            beam_drive.setDirection(DcMotor.Direction.FORWARD);
            beam_drive.setPower(0);
            beam_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            beam_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public static void sleep(double sec)
    {
        long ms = (long)(sec * 1000);
        try {
            Thread.sleep(ms);
        } catch (Exception e) {
            ;
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
