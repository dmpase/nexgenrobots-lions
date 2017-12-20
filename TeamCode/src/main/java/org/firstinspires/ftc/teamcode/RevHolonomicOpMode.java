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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import static com.sun.tools.javac.util.Constants.format;

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

@TeleOp(name="Rev Holonomic OpMode", group="Iterative Opmode")
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
    private double left_claw_open  = Servo.MIN_POSITION + 0.25 * (Servo.MAX_POSITION - Servo.MIN_POSITION);
    private double left_claw_mid   = Servo.MIN_POSITION + 0.50 * (Servo.MAX_POSITION - Servo.MIN_POSITION);
    private double left_claw_close = Servo.MIN_POSITION + 1.00 * (Servo.MAX_POSITION - Servo.MIN_POSITION);
    private Servo.Direction left_claw_dir = Servo.Direction.FORWARD;
    private double left_claw_del = 0;
    private double left_claw_pos = 0;

    private Servo right_claw = null;
    private double right_claw_open  = Servo.MIN_POSITION + 0.75 * (Servo.MAX_POSITION - Servo.MIN_POSITION);
    private double right_claw_mid   = Servo.MIN_POSITION + 0.50 * (Servo.MAX_POSITION - Servo.MIN_POSITION);
    private double right_claw_close = Servo.MIN_POSITION + 0.00 * (Servo.MAX_POSITION - Servo.MIN_POSITION);
    private Servo.Direction right_claw_dir = Servo.Direction.FORWARD;
    private double right_claw_del = 0;
    private double right_claw_pos = 0;

    private double claw_incr = 20;

    private DcMotor    lift     = null;
    private DcMotorEnc lift_ctl = null;

    //    private CRServo lift = null;
    private CRServo tail = null;

    // REV Robotics distance/color sensor
    private ColorSensor color_sensor = null;
    private DistanceSensor distance_sensor = null;

    private DistanceSensor mr_range = null;

    BNO055IMU imu0 = null;
    BNO055IMU imu1 = null;

    AnalogInput prs_lo = null;
    AnalogInput srs_lo = null;
    AnalogInput prs_hi = null;
    AnalogInput srs_hi = null;
    Distance    rs0d = new Distance(10.616758844230123, -2.625694922444332, 5.292315651154265);

    int cameraMonitorViewId = -1;
    VuforiaLocalizer vuforia = null;
    VuforiaLocalizer.Parameters vuforia_parameters = null;
    public static final String vuforia_license_key = "AWVXYZn/////AAAAGcG6g8XSSUMJsDaizcApOtsaA0fWzUQwImrdEn1MqH4JNqCzUwlyvEX0YALy7XyUeSpiANJkBg9kplUtcniUZKw8bF0dSpEfXZKXxn1yhbIohmpVmIK+Ngv1imYrkY6ePmvTfO2IpyQi5yO5ZmfSC8OzlH+XEMD0vRIXHMhxFpin7vTIHaoz8MEifSjRTznh1ZUSRnJfQ01KvMHEefES0kwhehlEKoqgpNMOYg0B5pV0bDDi9/Qh4eMR7sEk1GSx3QPxl/lYuZVcWSh8DutXv8oo9LhnbAaHTecCAR6gnNODow0WUAH2N9vxdLOjk2UfWVEJgqmHembIDHRzJN4fjcOECTFfLHIVmZ66GwgjPWxV";
    VuforiaTrackables relicTrackables = null;
    VuforiaTrackable relicTemplate = null;

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

        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_left.setDirection(DcMotor.Direction.FORWARD);
        front_left.setPower(0);
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        front_right = hardwareMap.get(DcMotor.class, "front_right");
        front_right.setDirection(DcMotor.Direction.FORWARD);
        front_right.setPower(0);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        back_right = hardwareMap.get(DcMotor.class, "back_right");
        back_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setPower(0);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_left.setDirection(DcMotor.Direction.FORWARD);
        back_left.setPower(0);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//        /*
        telemetry.addData("Status", "Initializing Servos.");

        left_claw  = hardwareMap.get(Servo.class, "left_claw");
        left_claw.setDirection(left_claw_dir);

        right_claw = hardwareMap.get(Servo.class, "right_claw");
        right_claw.setDirection(right_claw_dir);

        telemetry.addData("Status", "Initializing Lift & Tail.");

        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(DcMotor.Direction.FORWARD);
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_ctl = new DcMotorEnc(lift,0,0.10,0.01,500);

        tail = hardwareMap.get(CRServo.class, "tail");
        tail.setDirection(DcMotor.Direction.FORWARD);
        tail.setPower(0);

//        /*
        // REV Robotics distance/color sensor
        color_sensor    = hardwareMap.get(ColorSensor.class, "color range 2.1");
        distance_sensor = hardwareMap.get(DistanceSensor.class, "color range 2.1");
//        */

        mr_range = hardwareMap.get(DistanceSensor.class, "MR range 2.0");

        telemetry.addData("Status", "Initializing IMU.");

        BNO055IMU.Parameters imu_parameters = new BNO055IMU.Parameters();
        imu_parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imu_parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu_parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imu_parameters.loggingEnabled      = true;
        imu_parameters.loggingTag          = "IMU";
        imu_parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu0 = hardwareMap.get(BNO055IMU.class, "imu 0");
        imu0.initialize(imu_parameters);

        imu1 = hardwareMap.get(BNO055IMU.class, "imu 1");
        imu1.initialize(imu_parameters);

        prs_lo = hardwareMap.get(AnalogInput.class, "port_rs_low");
        srs_lo = hardwareMap.get(AnalogInput.class, "starboard_rs_low");
        prs_hi = hardwareMap.get(AnalogInput.class, "port_rs_high");
        srs_hi = hardwareMap.get(AnalogInput.class, "starboard_rs_high");

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

         /*/
         /*/
        telemetry.addData("Status", "Initializing VuForia.");
/*/
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        vuforia_parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vuforia_parameters.vuforiaLicenseKey = vuforia_license_key;
        vuforia_parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.createVuforiaLocalizer(vuforia_parameters);
        relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        relicTrackables.activate();
         /*/
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


        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

         /*/

//        telemetry.addData("VuMark",  " " + vuMark);
        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
        if (pose != null) {
            // Extract the X, Y, and Z components of the offset of the target relative to the robot
            VectorF trans = pose.getTranslation();

            double tX = trans.get(0);
            double tY = trans.get(1);
            double tZ = trans.get(2);

//            telemetry.addData("tXYZ", "%5.2fx %5.2fy %5.2fz", tX, tY, tZ);

            // Extract the rotational components of the target relative to the robot
            Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            double rX = rot.firstAngle;
            double rY = rot.secondAngle;
            double rZ = rot.thirdAngle;

//            telemetry.addData("rXYZ", "%5.2fx %5.2fy %5.2fz", rX, rY, rZ);
        }
         /*/

        telemetry.addData("Range", "%5.2fv %7.4fv %7.2f\"", prs_lo.getMaxVoltage(), prs_lo.getVoltage(),
                rs0d.distance(prs_lo.getVoltage()));

        telemetry.addData("Range", "%5.2fv %7.4fv %7.2f\"", srs_lo.getMaxVoltage(), srs_lo.getVoltage(),
                rs0d.distance(srs_lo.getVoltage()));

        telemetry.addData("Range", "%5.2fv %7.4fv %7.2f\"", prs_hi.getMaxVoltage(), prs_hi.getVoltage(),
                rs0d.distance(prs_hi.getVoltage()));

        telemetry.addData("Range", "%5.2fv %7.4fv %7.2f\"", srs_hi.getMaxVoltage(), srs_hi.getVoltage(),
                rs0d.distance(srs_hi.getVoltage()));

         /*/
        telemetry.addData("MR Range", "in=%6.2f",
                mr_range.getDistance(DistanceUnit.INCH));

        telemetry.addData("Motor Pos.", "%05d %05d %05d %05d",
                front_left.getCurrentPosition(), front_right.getCurrentPosition(),
                back_left .getCurrentPosition(), back_right .getCurrentPosition());

        telemetry.addData("Motor Power", "%5.2f %5.2f %5.2f %5.2f",
                motors[FRONT_LEFT], motors[FRONT_RIGHT],
                motors[BACK_LEFT], motors[BACK_RIGHT]);
         /*/

         /*/
        telemetry.addData("Claw Position", "%5.2f %5.2f %05d",
                servos[LEFT_CLAW], servos[RIGHT_CLAW], lift.getCurrentPosition());

        telemetry.addData("Lift Ctl", "%5.2f %5.2f %04d %04d %7d",
                lift_ctl.power, lift.getPower(), lift_ctl.target, lift.getCurrentPosition(), lift_ctl.count);

        telemetry.addData("Color/Dist", "cm=%6.2f a=%03d r=%03d g=%03d b=%03d",
                distance_sensor.getDistance(DistanceUnit.CM), color_sensor.alpha(),
                color_sensor.red(), color_sensor.green(), color_sensor.blue());
         /*/

        // Show the elapsed game time and other data.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    public static final int SERVO_COUNT = 4;
    public static final int TAIL        = 0;
    public static final int LIFT        = 1;
    public static final int LEFT_CLAW   = 2;
    public static final int RIGHT_CLAW  = 3;

    public static final int lift_max_pos =  5500;
    public static final int lift_mid_pos =     0;
    public static final int lift_min_pos =  -100;
    public static final double lift_max_pwr =  0.40;
    public static final double lift_mid_pwr =  0.00;
    public static final double lift_min_pwr = -lift_max_pwr;

    public void set_servo_power(double[] position)
    {
        if (position != null && position.length == SERVO_COUNT) {
            if (position[LEFT_CLAW] != left_claw_pos) {
                left_claw.setPosition(position[LEFT_CLAW]);
                left_claw_pos = position[LEFT_CLAW];
            }

            if (position[RIGHT_CLAW] != right_claw_pos) {
                right_claw.setPosition(position[RIGHT_CLAW]);
                right_claw_pos = position[RIGHT_CLAW];
            }

            tail.setPower(position[TAIL]);
            lift_ctl.update();
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
        } else if (gamepad1.b) {
            // open the claw
            servos[LEFT_CLAW]  = left_claw_open; // left_claw_pos  - left_claw_del;
            servos[RIGHT_CLAW] = right_claw_open; // right_claw_pos - right_claw_del;
        } else if (gamepad1.x) {
            // close the claw
            servos[LEFT_CLAW]  = left_claw_close; // left_claw_pos  + left_claw_del;
            servos[RIGHT_CLAW] = right_claw_close; // right_claw_pos + right_claw_del;
        } else {
            // maintain position, do nothing
            servos[LEFT_CLAW]  = left_claw_pos;
            servos[RIGHT_CLAW] = right_claw_pos;
        }

        servos[LEFT_CLAW]  = (servos[LEFT_CLAW]  < left_claw_open    ) ? left_claw_open   : servos[LEFT_CLAW];
        servos[LEFT_CLAW]  = (left_claw_close    < servos[LEFT_CLAW] ) ? left_claw_close  : servos[LEFT_CLAW];
        servos[RIGHT_CLAW] = (servos[RIGHT_CLAW] < right_claw_open   ) ? right_claw_open  : servos[RIGHT_CLAW];
        servos[RIGHT_CLAW] = (right_claw_close   < servos[RIGHT_CLAW]) ? right_claw_close : servos[RIGHT_CLAW];

        if (gamepad1.y && gamepad1.a) {
            lift_ctl.setTargetPosition(lift_mid_pos);
        } else if (gamepad1.y) {
            lift_ctl.setTargetPosition(lift_max_pos);
        } else if (gamepad1.a) {
            lift_ctl.setTargetPosition(lift_min_pos);
        } else {
            ;
        }

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

    public static final double stick_dead_zone   = 0.05;
    public static final double full_speed        = 0.95;
    public static final double half_speed        = 0.50;
    public static final double slow_speed        = 0.25;

    public static final double trigger_dead_zone = 0.05;
    public static final double full_turn         = 0.40;
    public static final double half_turn         = 0.20;
    public static final double slow_turn         = 0.10;
    public static final double turn_limit        = 0.75;


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
        // use game pad 1 right stick to determinie speed and bearing UNLESS the left stick is being used
        // right stick is for speed, left stick is for precision
        double x = (-stick_dead_zone < gamepad1.right_stick_x && gamepad1.right_stick_x < stick_dead_zone) ? 0 : full_speed * gamepad1.right_stick_x;
        double y = (-stick_dead_zone < gamepad1.right_stick_y && gamepad1.right_stick_y < stick_dead_zone) ? 0 : full_speed * gamepad1.right_stick_y;
        if (gamepad1.left_stick_x < -stick_dead_zone || stick_dead_zone < gamepad1.left_stick_x ||
            gamepad1.left_stick_y < -stick_dead_zone || stick_dead_zone < gamepad1.left_stick_y) {

            x = half_speed * gamepad1.left_stick_x;
            y = half_speed * gamepad1.left_stick_y;
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
            throttle = slow_speed;
            bearing = 0 * Math.PI/4;
        } else if (gamepad1.dpad_up && gamepad1.dpad_right && ! gamepad1.dpad_down && ! gamepad1.dpad_left) {
            throttle = slow_speed;
            bearing = 1 * Math.PI/4;
        } else if (! gamepad1.dpad_up && gamepad1.dpad_right && ! gamepad1.dpad_down && ! gamepad1.dpad_left) {
            throttle = slow_speed;
            bearing = 2 * Math.PI/4;
        } else if (! gamepad1.dpad_up && gamepad1.dpad_right && gamepad1.dpad_down && ! gamepad1.dpad_left) {
            throttle = slow_speed;
            bearing = 3 * Math.PI/4;
        } else if (! gamepad1.dpad_up && ! gamepad1.dpad_right && gamepad1.dpad_down && ! gamepad1.dpad_left) {
            throttle = slow_speed;
            bearing = 4 * Math.PI/4;
        } else if (! gamepad1.dpad_up && ! gamepad1.dpad_right && gamepad1.dpad_down && gamepad1.dpad_left) {
            throttle = slow_speed;
            bearing = 5 * Math.PI/4;
        } else if (! gamepad1.dpad_up && ! gamepad1.dpad_right && ! gamepad1.dpad_down && gamepad1.dpad_left) {
            throttle = slow_speed;
            bearing = 6 * Math.PI/4;
        } else if (gamepad1.dpad_up && ! gamepad1.dpad_right && ! gamepad1.dpad_down && gamepad1.dpad_left) {
            throttle = slow_speed;
            bearing = 7 * Math.PI/4;
        }

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
