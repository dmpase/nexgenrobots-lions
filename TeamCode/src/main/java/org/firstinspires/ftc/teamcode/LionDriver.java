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

import static com.sun.tools.javac.util.Constants.format;

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

@TeleOp(name="Lion Driver OpMode", group="Iterative OpMode")
// @Disabled
public class LionDriver extends OpMode
{
    // Declare OpMode members.
    // REV Robotics drive motors
    private DcMotor port_bow_motor = null;
    private DcMotor stbd_bow_motor = null;
    private DcMotor stbd_aft_motor = null;
    private DcMotor port_aft_motor = null;

    // claw and tail servos
    private Servo port_claw = null;
    private Servo stbd_claw = null;

    // claw lift and beam motors
    private DcMotor lift = null;
    private DcMotor beam = null;

    private ElapsedTime runtime = new ElapsedTime();

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
        init_motors();

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
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }


    public void get_beam_settings()
    {
        if (gamepad1.dpad_left && ! gamepad1.dpad_right) {           // extend the beam
            beam.setPower(0);
            beam.setTargetPosition(Config.BEAM_TARGET_OUT);
            beam.setPower(Config.BEAM_POWER);

            while ( Config.MOTOR_TARGET_TOLERANCE < Math.abs(beam.getTargetPosition() - beam.getCurrentPosition()) ) {
                ;
            }

            sleep(Config.MOTOR_LAG);

            beam.setPower(0);
        } else if (! gamepad1.dpad_left && gamepad1.dpad_right) {    // retract the beam
            beam.setPower(0);
            beam.setTargetPosition(Config.BEAM_TARGET_IN);
            beam.setPower(Config.BEAM_POWER);

            while ( Config.MOTOR_TARGET_TOLERANCE < Math.abs(beam.getTargetPosition() - beam.getCurrentPosition()) ) {
                ;
            }

            sleep(Config.MOTOR_LAG);

            beam.setPower(0);
        }
    }


    public static final int TAIL        = 0;
    public static final int LIFT        = 1;
    public static final int PORT_CLAW   = 2;
    public static final int STBD_CLAW   = 3;
    public static final int SERVO_COUNT = 4;

    public static final int    lift_max_pos =  5500;
    public static final int    lift_mid_pos =     0;
    public static final int    lift_min_pos =  -100;
    public static final double lift_max_pwr =  0.40;
    public static final double lift_mid_pwr =  0.00;
    public static final double lift_min_pwr = -lift_max_pwr;

    public void get_claw_settings()
    {
        if (gamepad1.x && ! gamepad1.b) {           // open the claw
            port_claw.setPosition(Config.PORT_CLAW_OPENED);
            stbd_claw.setPosition(Config.STBD_CLAW_OPENED);
        } else if (! gamepad1.x && gamepad1.b) {    // close the claw
            port_claw.setPosition(Config.PORT_CLAW_CLOSED);
            stbd_claw.setPosition(Config.STBD_CLAW_CLOSED);
        }

        if (gamepad1.y && ! gamepad1.a) {           // raise the claw
            lift.setPower(0);
            lift.setTargetPosition(Config.LIFT_TARGET_HI);
            lift.setPower(Config.LIFT_POWER);

            while ( Config.MOTOR_TARGET_TOLERANCE < Math.abs(lift.getTargetPosition() - lift.getCurrentPosition()) ) {
                ;
            }

            sleep(Config.MOTOR_LAG);

            lift.setPower(0);
        } else if (! gamepad1.y && gamepad1.a) {    // lower the claw
            lift.setPower(0);
            lift.setTargetPosition(Config.LIFT_TARGET_LO);
            lift.setPower(Config.LIFT_POWER);

            while ( Config.MOTOR_TARGET_TOLERANCE < Math.abs(lift.getTargetPosition() - lift.getCurrentPosition()) ) {
                ;
            }

            sleep(Config.MOTOR_LAG);

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

        port_bow_motor.setPower(motors[PORT_BOW]);
        stbd_bow_motor.setPower(motors[STBD_BOW]);
        stbd_aft_motor.setPower(motors[STBD_AFT]);
        port_aft_motor.setPower(motors[PORT_AFT]);
    }

    private void init_motors() {
        if (port_bow_motor == null) {
            port_bow_motor = hardwareMap.get(DcMotor.class, Config.PORT_BOW);
            port_bow_motor.setDirection(DcMotor.Direction.FORWARD);
            port_bow_motor.setPower(0);
            port_bow_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            port_bow_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (stbd_bow_motor == null) {
            stbd_bow_motor = hardwareMap.get(DcMotor.class, Config.STBD_BOW);
            stbd_bow_motor.setDirection(DcMotor.Direction.FORWARD);
            stbd_bow_motor.setPower(0);
            stbd_bow_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            stbd_bow_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (stbd_aft_motor == null) {
            stbd_aft_motor = hardwareMap.get(DcMotor.class, Config.STBD_AFT);
            stbd_aft_motor.setDirection(DcMotor.Direction.FORWARD);
            stbd_aft_motor.setPower(0);
            stbd_aft_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            stbd_aft_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (port_aft_motor == null) {
            port_aft_motor = hardwareMap.get(DcMotor.class, Config.PORT_AFT);
            port_aft_motor.setDirection(DcMotor.Direction.FORWARD);
            port_aft_motor.setPower(0);
            port_aft_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            port_aft_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void init_claw()
    {
        if (port_claw == null) {
            port_claw = hardwareMap.get(Servo.class, Config.PORT_CLAW);
            port_claw.setDirection(Servo.Direction.FORWARD);
        }

        if (stbd_claw == null) {
            stbd_claw = hardwareMap.get(Servo.class, Config.STBD_CLAW);
            stbd_claw.setDirection(Servo.Direction.FORWARD);
        }

        if (lift == null) {
            lift = hardwareMap.get(DcMotor.class, Config.LIFT);
            lift.setDirection(Config.LIFT_DIRECTION);
            lift.setPower(0);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void init_beam()
    {
        if (beam == null) {
            beam = hardwareMap.get(DcMotor.class, Config.BEAM);
            beam.setDirection(DcMotor.Direction.FORWARD);
            beam.setPower(0);
            beam.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            beam.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    public void sleep(double sec)
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
