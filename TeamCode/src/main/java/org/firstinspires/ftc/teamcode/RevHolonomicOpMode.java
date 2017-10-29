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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

import java.util.Iterator;
import java.util.List;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just echoes input from the controllers.
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="RevHolonomicOpMode", group="Iterative Opmode")
// @Disabled
public class RevHolonomicOpMode extends OpMode
{
    // Declare OpMode members.
    // REV Robotics drive motors
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_right  = null;
    private DcMotor back_left   = null;

    private Servo left_claw  = null;
    private double left_claw_min = 0;
    private double left_claw_max = 0;
    private double left_claw_del = 0;
    private double left_claw_pos = 0;

    private Servo right_claw = null;
    private double right_claw_min = 0;
    private double right_claw_max = 0;
    private double right_claw_del = 0;
    private double right_claw_pos = 0;

    private double claw_incr = 20;

    //    private CRServo lift = null;
    private CRServo tail = null;

    // REV Robotics distance/color sensor
//    private ColorSensor    color_sensor;
//    private DistanceSensor distance_sensor;

    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()
    {
        telemetry.addData("Status", "Starting Initialization.");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        telemetry.addData("Status", "Initializing Motors.");

        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_left.setDirection(DcMotor.Direction.FORWARD);
        front_left.setPower(0);

        front_right = hardwareMap.get(DcMotor.class, "front_right");
        front_right.setDirection(DcMotor.Direction.FORWARD);
        front_right.setPower(0);

        back_right = hardwareMap.get(DcMotor.class, "back_right");
        back_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setPower(0);

        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_left.setDirection(DcMotor.Direction.FORWARD);
        back_left.setPower(0);


        /*
        telemetry.addData("Status", "Initializing Servos.");
        left_claw = hardwareMap.get(Servo.class, "left_claw");
        left_claw.setDirection(Servo.Direction.FORWARD);
        left_claw.setPosition(Servo.MIN_POSITION);
        left_claw_min = left_claw.getPosition();
        left_claw.setPosition(Servo.MAX_POSITION);
        left_claw_max = left_claw.getPosition();
        left_claw_del = (left_claw_max - left_claw_min) / claw_incr;
        left_claw_pos = left_claw_min + left_claw_del;
        left_claw.setPosition(right_claw_pos);

        right_claw = hardwareMap.get(Servo.class, "right_claw");
        left_claw.setDirection(Servo.Direction.FORWARD);
        right_claw.setPosition(Servo.MIN_POSITION);
        right_claw_min = right_claw.getPosition();
        right_claw.setPosition(Servo.MAX_POSITION);
        right_claw_max = right_claw.getPosition();
        right_claw_del = (right_claw_max - right_claw_min) / claw_incr;
        right_claw_pos = right_claw_max - right_claw_del;
        right_claw.setPosition(right_claw_pos);
        */

        telemetry.addData("Status", "Initializing CR Servos.");

        /*
        lift = hardwareMap.get(CRServo.class, "lift");
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setPower(0);
        */

        tail = hardwareMap.get(CRServo.class, "tail");
        tail.setDirection(DcMotor.Direction.FORWARD);
        tail.setPower(0);

        /*
        // REV Robotics distance/color sensor
        color_sensor    = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        distance_sensor = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        */

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
        double[] motors = compute_motor_settings();
        set_motor_power(motors);

        double[] servos = compute_servo_settings();
        set_servo_power(servos);

        // Show the elapsed game time and wheel power.
        telemetry.addData("", "fl = %.2f  fr = %.2f  bl = %.2f  br = %.2f",
                motors[FRONT_LEFT], motors[FRONT_RIGHT],
                motors[BACK_LEFT], motors[BACK_RIGHT]);

        telemetry.addData("", "min = %.2f lc = %.2f  rc = %.2f max = %.2f",
                Servo.MIN_POSITION, servos[LEFT_CLAW], servos[RIGHT_CLAW], Servo.MAX_POSITION);

        /*
        telemetry.addData("", "cm=%.2f a=%d r=%d g=%d b=%d",
                distance_sensor.getDistance(DistanceUnit.CM), color_sensor.alpha(),
                color_sensor.red(), color_sensor.green(), color_sensor.blue());
        */

        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    public static final int SERVO_COUNT = 4;
    public static final int TAIL        = 0;
    public static final int LIFT        = 1;
    public static final int LEFT_CLAW   = 2;
    public static final int RIGHT_CLAW  = 3;

    public void set_servo_power(double[] position)
    {
        if (position != null && position.length == SERVO_COUNT) {
            if (position[LEFT_CLAW] != left_claw_pos) {
                left_claw .setPosition(position[LEFT_CLAW]);
                left_claw_pos = position[LEFT_CLAW];
            }

            if (position[RIGHT_CLAW] != right_claw_pos) {
                right_claw.setPosition(position[RIGHT_CLAW]);
                right_claw_pos = position[RIGHT_CLAW];
            }

            tail.setPower(position[TAIL]);
//            lift.setPower(position[LIFT]);
        }
    }

    public double[] compute_servo_settings()
    {
        double[] servos = new double[SERVO_COUNT];

        // open or close the claw
        if (gamepad1.x && gamepad1.b) {
            // conflicting inputs, do nothing
            servos[LEFT_CLAW]  = left_claw_pos;
            servos[RIGHT_CLAW] = right_claw_pos;
        } else if (gamepad1.x) {
            // close the claw
            servos[LEFT_CLAW]  = left_claw_pos  + left_claw_del;
            servos[RIGHT_CLAW] = right_claw_pos + right_claw_del;
        } else if (gamepad1.b) {
            // close the claw
            servos[LEFT_CLAW]  = left_claw_pos  - left_claw_del;
            servos[RIGHT_CLAW] = right_claw_pos - right_claw_del;
        } else {
            // maintain position, do nothing
            servos[LEFT_CLAW]  = left_claw_pos;
            servos[RIGHT_CLAW] = right_claw_pos;
        }
        servos[LEFT_CLAW]  = (servos[LEFT_CLAW] < left_claw_min) ? left_claw_min : left_claw_pos;
        servos[LEFT_CLAW]  = (left_claw_max < servos[LEFT_CLAW]) ? left_claw_max : left_claw_pos;
        servos[RIGHT_CLAW]  = (servos[RIGHT_CLAW] < right_claw_min) ? right_claw_min : right_claw_pos;
        servos[RIGHT_CLAW]  = (right_claw_max < servos[RIGHT_CLAW]) ? right_claw_max : right_claw_pos;

        /*
        if (gamepad1.y) {
            servos[LIFT] = 0.10;
        } else if (gamepad1.a) {
            servos[LIFT] = -0.10;
        } else {
            servos[LIFT] = 0;
        }
        */

        if (gamepad1.back) {
            servos[TAIL] = 0.10;
        } else if (gamepad1.start) {
            servos[TAIL] = -0.10;
        } else {
            servos[TAIL] = 0;
        }

        return servos;
    }

    public static final int MOTOR_COUNT = 4;
    public static final int FRONT_LEFT  = 0;
    public static final int FRONT_RIGHT = 1;
    public static final int BACK_RIGHT  = 2;
    public static final int BACK_LEFT   = 3;

    public static final double POWER_LIMIT = 0.95;

    public void set_motor_power(double[] power)
    {
        if (power != null && power.length == MOTOR_COUNT) {
            front_left .setPower(power[FRONT_LEFT ]);
            front_right.setPower(power[FRONT_RIGHT]);
            back_right .setPower(power[BACK_RIGHT ]);
            back_left  .setPower(power[BACK_LEFT  ]);
        }
    }

    public double[] compute_motor_settings()
    {
        final double stick_dead_zone   = 0.05;
        final double precision_speed   = 0.50;

        final double trigger_dead_zone = 0.05;
        final double precision_turn    = 0.20;
        final double turn_limit        = 0.75;

        // use game pad 1 right stick to determinie speed and bearing UNLESS the left stick is being used
        // right stick is for speed, left stick is for precision
        double x = (-stick_dead_zone < gamepad1.right_stick_x && gamepad1.right_stick_x < stick_dead_zone) ? 0 : gamepad1.right_stick_x;
        double y = (-stick_dead_zone < gamepad1.right_stick_y && gamepad1.right_stick_y < stick_dead_zone) ? 0 : gamepad1.right_stick_y;
        if (gamepad1.left_stick_x < -stick_dead_zone || stick_dead_zone < gamepad1.left_stick_x ||
            gamepad1.left_stick_y < -stick_dead_zone || stick_dead_zone < gamepad1.left_stick_y) {

            x = precision_speed * gamepad1.left_stick_x;
            y = precision_speed * gamepad1.left_stick_y;
        }

        // get the angle of the stick to compute the desired bearing
        double alpha   = (x == 0 && y == 0) ? 0 : Math.atan2(y, x);
        double bearing = (x == 0 && y == 0) ? 0 : alpha + Math.PI/2.0;

        // limit the throttle
        double throttle = Math.sqrt(x*x + y*y);
        throttle = (POWER_LIMIT < throttle) ? POWER_LIMIT : throttle;

        // enable the dpad for movement
        // dpad overrides both sticks
        if (gamepad1.dpad_up && ! gamepad1.dpad_right && ! gamepad1.dpad_down && ! gamepad1.dpad_left) {
            throttle = precision_speed;
            bearing = 0 * Math.PI/4;
        } else if (gamepad1.dpad_up && gamepad1.dpad_right && ! gamepad1.dpad_down && ! gamepad1.dpad_left) {
            throttle = precision_speed;
            bearing = 1 * Math.PI/4;
        } else if (! gamepad1.dpad_up && gamepad1.dpad_right && ! gamepad1.dpad_down && ! gamepad1.dpad_left) {
            throttle = precision_speed;
            bearing = 2 * Math.PI/4;
        } else if (! gamepad1.dpad_up && gamepad1.dpad_right && gamepad1.dpad_down && ! gamepad1.dpad_left) {
            throttle = precision_speed;
            bearing = 3 * Math.PI/4;
        } else if (! gamepad1.dpad_up && ! gamepad1.dpad_right && gamepad1.dpad_down && ! gamepad1.dpad_left) {
            throttle = precision_speed;
            bearing = 4 * Math.PI/4;
        } else if (! gamepad1.dpad_up && ! gamepad1.dpad_right && gamepad1.dpad_down && gamepad1.dpad_left) {
            throttle = precision_speed;
            bearing = 5 * Math.PI/4;
        } else if (! gamepad1.dpad_up && ! gamepad1.dpad_right && ! gamepad1.dpad_down && gamepad1.dpad_left) {
            throttle = precision_speed;
            bearing = 6 * Math.PI/4;
        } else if (gamepad1.dpad_up && ! gamepad1.dpad_right && ! gamepad1.dpad_down && gamepad1.dpad_left) {
            throttle = precision_speed;
            bearing = 7 * Math.PI/4;
        }

        // add in rotation, if any
        double rotation = 0;
        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            // conflicting inputs, do nothing
        } else if (gamepad1.left_bumper) {
            // bumpers take priority, turn left
            rotation = precision_turn;
        } else if (gamepad1.right_bumper) {
            // bumpers take priority, turn right
            rotation = -precision_turn;
        } else if (trigger_dead_zone < gamepad1.left_trigger && trigger_dead_zone < gamepad1.right_trigger) {
            // conflicting inputs, do nothing
        } else if (trigger_dead_zone < gamepad1.left_trigger) {
            // turn left using trigger
            rotation = turn_limit * gamepad1.left_trigger;
        } else if (trigger_dead_zone < gamepad1.right_trigger) {
            // turn right using trigger
            rotation = -turn_limit * gamepad1.right_trigger;
        }

        double[] motors = new double[MOTOR_COUNT];

        motors[FRONT_LEFT ] = - throttle * Math.sin(bearing + Math.PI/4) + rotation;
        motors[FRONT_RIGHT] =   throttle * Math.cos(bearing + Math.PI/4) + rotation;
        motors[BACK_RIGHT ] =   throttle * Math.sin(bearing + Math.PI/4) + rotation;
        motors[BACK_LEFT  ] = - throttle * Math.cos(bearing + Math.PI/4) + rotation;

        // limit the motors to -1.0 <= motor <= 1.0
        // scale them evenly if adjustments are made
        if (motors[FRONT_LEFT ] < -POWER_LIMIT || POWER_LIMIT < motors[FRONT_LEFT ] ||
            motors[FRONT_RIGHT] < -POWER_LIMIT || POWER_LIMIT < motors[FRONT_RIGHT] ||
            motors[BACK_LEFT  ] < -POWER_LIMIT || POWER_LIMIT < motors[BACK_LEFT  ] ||
            motors[BACK_RIGHT ] < -POWER_LIMIT || POWER_LIMIT < motors[BACK_RIGHT ]) {

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

        return motors;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
