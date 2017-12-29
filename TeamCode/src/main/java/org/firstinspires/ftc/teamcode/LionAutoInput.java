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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="Lion Auto Input", group="Autonomous")
// @Disabled
public class LionAutoInput extends LinearOpMode {
    // Declare OpMode members.

    public static enum Command {ROTATE, FORWARD, BACKWARD, LEFT, RIGHT, ADJUST, OPEN_CLAW, CLOSE_CLAW, LIFT}
    public static enum Color {BLUE, RED, UNKNOWN}

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        telemetry.addData("Status", "Starting Initialization.");

        motor_init();

        servo_init();

        sensor_init();

        vuforia_init();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Status", "Initialization Complete.");
        telemetry.update();

        telemetry.addData("Claw", "'X' to open, 'B' to close.");
        telemetry.addData("Lift", "'Y' to raise, 'A' to lower.");
        telemetry.addData("Tail", "DPAD 'Up' to raise, 'Down' to lower.");
        telemetry.addData("Start", "'Guide' to select quadrant.");
        telemetry.update();
        Object[] open_claw  = {Command.OPEN_CLAW,                           };
        Object[] close_claw = {Command.CLOSE_CLAW,                          };
        Object[] raise_claw = {Command.LIFT,        Config.LIFT_TARGET_INCH };
        Object[] lower_claw = {Command.LIFT,        Config.LIFT_TARGET_LO   };
        while (! gamepad1.guide) {
            if (gamepad1.x) {
                execute(open_claw);
            } else if (gamepad1.b) {
                execute(close_claw);
            } else if (gamepad1.y) {
                execute(raise_claw);
            } else if (gamepad1.a) {
                execute(lower_claw);
            } else if (gamepad1.dpad_up) {
                tail.setPosition(Config.TAIL_POS_UP);
            } else if (gamepad1.dpad_down) {
                tail.setPosition(Config.TAIL_POS_DN);
            }
        }


        Color team_color = Color.UNKNOWN;
        int quadrant = UNKNOWN;

        telemetry.addData("Blue", "'X' for left, 'A' for right.");
        telemetry.addData("Red", "'Y' for left, 'B' for right.");
        telemetry.addData("Start", "'Start' to wait for start.");
        telemetry.update();
        while (! gamepad1.start) {
            if (gamepad1.x) {
                quadrant = BLUE_LEFT;
                team_color = Color.BLUE;
            } else if (gamepad1.a) {
                quadrant = BLUE_RIGHT;
                team_color = Color.BLUE;
            } else if (gamepad1.y) {
                quadrant = RED_LEFT;
                team_color = Color.RED;
            } else if (gamepad1.b) {
                quadrant = RED_RIGHT;
                team_color = Color.RED;
            }
        }


        telemetry.addData("Quadrant", "%s(%d) %s",
                QUADRANT_NAME[quadrant], quadrant, team_color.toString());
        telemetry.addData("Status", "Ready to play, waiting for start.");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();


        // ^^^ vuforia

        // Vuforia reading: left, center, or right; left = -1, center = 0, right = 1;
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        String vumark_position = vuMark.name();
        if (vumark_position.equalsIgnoreCase("LEFT")) {
            vuforiaresult = VUFORIA_LEFT;
        } else if (vumark_position.equalsIgnoreCase("CENTER")) {
            vuforiaresult = VUFORIA_CENTER;
        } else if (vumark_position.equalsIgnoreCase("RIGHT")) {
            vuforiaresult = VUFORIA_RIGHT;
        }

        telemetry.addData("VuMark", "%s", vuMark.name());


        // ^^^ dislodge the jewell_color (or not...)
        // lower tail
        tail.setPosition(Config.TAIL_POS_DN);
        sleep(2000);

        // read the jewell color
        Color jewell_color = Color.UNKNOWN;
        if (color_sensor.red() < color_sensor.blue()) {
            jewell_color = Color.BLUE;
        } else if (color_sensor.blue() < color_sensor.red()) {
            jewell_color = Color.RED;
        }

        // if jewell color == team color, clockwise then counter clockwise, else opposite
        if (team_color == Color.UNKNOWN || jewell_color == Color.UNKNOWN) {
            ;
        } else if (team_color == jewell_color) {
            Object[][] cmd = {
                    {Command.ROTATE, +30.0, AUTO_PWR, AUTO_TOL},
                    {Command.ROTATE, -30.0, AUTO_PWR, AUTO_TOL},
            };

            execute(cmd);
        } else {
            Object[][] cmd = {
                    {Command.ROTATE, -30.0, AUTO_PWR, AUTO_TOL},
                    {Command.ROTATE, +30.0, AUTO_PWR, AUTO_TOL},
            };

            execute(cmd);
        }

        // raise tail
        tail.setPosition(Config.TAIL_POS_UP);
        sleep(1000);


        // ^^^ deliver the block to the crypto box
        execute(quad_cmds[quadrant]);

        telemetry.addData("Path", "Complete");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }


    private static final int UNKNOWN    = 0;
    private static final int BLUE_LEFT  = 1;
    private static final int BLUE_RIGHT = 2;
    private static final int RED_LEFT   = 3;
    private static final int RED_RIGHT  = 4;
    private static final String[] QUADRANT_NAME = {"UNKNOWN", "BLUE LEFT", "BLUE RIGHT", "RED LEFT", "RED RIGHT"};

    private static final double AUTO_PWR = 0.1;
    private static final int    AUTO_TOL = 10;

    private static final Object[][] blue_left_cmd = {
            {Command.ROTATE,    90.0, AUTO_PWR, AUTO_TOL},
            {Command.FORWARD,   36.0, AUTO_PWR, AUTO_TOL},
            {Command.ROTATE,    90.0, AUTO_PWR, AUTO_TOL},
            {Command.ADJUST,     8.0, AUTO_PWR, AUTO_TOL},
            {Command.FORWARD,   18.0, AUTO_PWR, AUTO_TOL},
            {Command.OPEN_CLAW,                         },
            {Command.BACKWARD,   8.0, AUTO_PWR, AUTO_TOL},
            {Command.CLOSE_CLAW,                        },
            {Command.LIFT,        Config.LIFT_TARGET_LO },
    };

    private static final Object[][] blue_right_cmd = {
            {Command.ROTATE,    90.0, AUTO_PWR, AUTO_TOL},
            {Command.FORWARD,   36.0, AUTO_PWR, AUTO_TOL},
            {Command.RIGHT,     12.0, AUTO_PWR, AUTO_TOL},
            {Command.ADJUST,     8.0, AUTO_PWR, AUTO_TOL},
            {Command.FORWARD,   12.0, AUTO_PWR, AUTO_TOL},
            {Command.OPEN_CLAW,                         },
            {Command.BACKWARD,   8.0, AUTO_PWR, AUTO_TOL},
            {Command.CLOSE_CLAW,                        },
            {Command.LIFT,        Config.LIFT_TARGET_LO },
    };

    private static final Object[][] red_left_cmd = {
            {Command.ROTATE,   -90.0, AUTO_PWR, AUTO_TOL},
            {Command.FORWARD,   36.0, AUTO_PWR, AUTO_TOL},
            {Command.ROTATE,   -90.0, AUTO_PWR, AUTO_TOL},
            {Command.ADJUST,     8.0, AUTO_PWR, AUTO_TOL},
            {Command.FORWARD,   18.0, AUTO_PWR, AUTO_TOL},
            {Command.OPEN_CLAW,                         },
            {Command.BACKWARD,   8.0, AUTO_PWR, AUTO_TOL},
            {Command.CLOSE_CLAW,                        },
            {Command.LIFT,        Config.LIFT_TARGET_LO },
    };

    private static final Object[][] red_right_cmd = {
            {Command.ROTATE,   -90.0, AUTO_PWR, AUTO_TOL},
            {Command.FORWARD,   36.0, AUTO_PWR, AUTO_TOL},
            {Command.LEFT,      12.0, AUTO_PWR, AUTO_TOL},
            {Command.ADJUST,     8.0, AUTO_PWR, AUTO_TOL},
            {Command.ADJUST,     8.0, AUTO_PWR, AUTO_TOL},
            {Command.FORWARD,   12.0, AUTO_PWR, AUTO_TOL},
            {Command.OPEN_CLAW,                         },
            {Command.BACKWARD,   8.0, AUTO_PWR, AUTO_TOL},
            {Command.CLOSE_CLAW,                        },
            {Command.LIFT,        Config.LIFT_TARGET_LO },
    };

    private static final Object[][][] quad_cmds = {
            null,
            blue_left_cmd,
            blue_right_cmd,
            red_left_cmd,
            red_right_cmd,
    };

    private static final int OPCODE     = 0;
    private static final int ANGLE      = 1;
    private static final int INCHES     = 1;
    private static final int TARGET     = 1;
    private static final int POWER      = 2;
    private static final int TOLERANCE  = 3;

    private static final int VUFORIA_LEFT   = -1;
    private static final int VUFORIA_CENTER = 0;
    private static final int VUFORIA_RIGHT  = 1;
    private int vuforiaresult = VUFORIA_CENTER;

    private void execute(Object[][] cmd)
    {
        for (int i=0; cmd != null && i < cmd.length; i++) {
            execute(cmd[i]);
        }
    }

    private void execute(Object[] cmd)
    {
        Command op_code = (Command) cmd[OPCODE];
        if (op_code == Command.ROTATE) {
            double angle     = (double) cmd[ANGLE];
            double power     = (double) cmd[POWER];
            int    tolerance = (int)    cmd[TOLERANCE];
            // 90 degrees == 900 clicks, or clicks = 10 * angle in degrees
            int clicks = (int) (10 * angle);
            run_to_position(clicks, clicks, clicks, clicks, power, tolerance);
        } else if (op_code == Command.FORWARD) {
            double distance  = (double) cmd[INCHES];
            double power     = (double) cmd[POWER];
            int    tolerance = (int)    cmd[TOLERANCE];
            // 15 inches == 750 clicks, or clicks = 50 * distance in inches
            int clicks = (int) (50 * distance);
            run_to_position(-clicks, clicks, clicks, -clicks, power, tolerance);
        } else if (op_code == Command.BACKWARD) {
            double distance  = (double) cmd[INCHES];
            double power     = (double) cmd[POWER];
            int    tolerance = (int)    cmd[TOLERANCE];
            // 15 inches == 750 clicks, or clicks = 50 * distance in inches
            int clicks = (int) (50 * distance);
            run_to_position(clicks, -clicks, -clicks, clicks, power, tolerance);
        } else if (op_code == Command.LEFT) {
            double distance  = (double) cmd[INCHES];
            double power     = (double) cmd[POWER];
            int    tolerance = (int)    cmd[TOLERANCE];
            // 15 inches == 750 clicks, or clicks = 50 * distance in inches
            int clicks = (int) (50 * distance);
            run_to_position(clicks, clicks, -clicks, -clicks, power, tolerance);
        } else if (op_code == Command.RIGHT) {
            double distance  = (double) cmd[INCHES];
            double power     = (double) cmd[POWER];
            int    tolerance = (int)    cmd[TOLERANCE];
            // 15 inches == 750 clicks, or clicks = 50 * distance in inches
            int clicks = (int) (50 * distance);
            run_to_position(-clicks, -clicks, clicks, clicks, power, tolerance);
        } else if (op_code == Command.ADJUST) {
            double distance  = vuforiaresult * (double) cmd[INCHES];
            double power     = (double) cmd[POWER];
            int    tolerance = (int)    cmd[TOLERANCE];
            // 15 inches == 750 clicks, or clicks = 50 * distance in inches
            int clicks = (int) (50 * distance);
            run_to_position(-clicks, clicks, clicks, -clicks, power, tolerance);
        } else if (op_code == Command.OPEN_CLAW) {
            port_claw.setPosition(Config.PORT_CLAW_OPENED);
            stbd_claw.setPosition(Config.STBD_CLAW_OPENED);
            sleep(Config.MOTOR_LAG);
        } else if (op_code == Command.CLOSE_CLAW) {
            port_claw.setPosition(Config.PORT_CLAW_CLOSED);
            stbd_claw.setPosition(Config.STBD_CLAW_CLOSED);
            sleep(Config.MOTOR_LAG);
        } else if (op_code == Command.LIFT) {
            int target = (int) cmd[TARGET];

            lift.setPower(0);
            lift.setTargetPosition(target);
            lift.setPower(Config.LIFT_POWER);

            while ( Config.MOTOR_TARGET_TOLERANCE < Math.abs(lift.getTargetPosition() - lift.getCurrentPosition()) ) {
                ;
            }

            sleep(Config.MOTOR_LAG);

            lift.setPower(0);
        }
    }

    private DcMotor port_bow_motor = null;
    private DcMotor stbd_bow_motor = null;
    private DcMotor stbd_aft_motor = null;
    private DcMotor port_aft_motor = null;

    public void motor_init()
    {
        telemetry.addData("Status", "Initializing Motors.");

        port_bow_motor = hardwareMap.get(DcMotor.class, Config.PORT_BOW);
        stbd_bow_motor = hardwareMap.get(DcMotor.class, Config.STBD_BOW);
        stbd_aft_motor = hardwareMap.get(DcMotor.class, Config.STBD_AFT);
        port_aft_motor = hardwareMap.get(DcMotor.class, Config.PORT_AFT);

        port_bow_motor.setPower(0);
        stbd_bow_motor.setPower(0);
        stbd_aft_motor.setPower(0);
        port_aft_motor.setPower(0);

        port_bow_motor.setDirection(DcMotor.Direction.FORWARD);
        stbd_bow_motor.setDirection(DcMotor.Direction.FORWARD);
        stbd_aft_motor.setDirection(DcMotor.Direction.FORWARD);
        port_aft_motor.setDirection(DcMotor.Direction.FORWARD);

        port_bow_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stbd_bow_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stbd_aft_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        port_aft_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    // claw and tail servos
    private Servo port_claw = null;
    private Servo stbd_claw = null;
    private Servo tail      = null;

    // claw lift motor
    private DcMotor lift    = null;

    public void servo_init()
    {
        port_claw = hardwareMap.get(Servo.class, Config.PORT_CLAW);
        port_claw.setDirection(Servo.Direction.FORWARD);

        stbd_claw = hardwareMap.get(Servo.class, Config.STBD_CLAW);
        stbd_claw.setDirection(Servo.Direction.FORWARD);

        tail  = hardwareMap.get(Servo.class, Config.TAIL);
        tail.setDirection(Servo.Direction.FORWARD);

        lift = hardwareMap.get(DcMotor.class, Config.LIFT);
        lift.setDirection(Config.LIFT_DIRECTION);
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }



    // REV Robotics distance/color sensor
    private ColorSensor color_sensor = null;
    private DistanceSensor distance_sensor = null;

    public void sensor_init()
    {
        telemetry.addData("Status", "Initializing Sensors.");

        // tail Rev color/distance sensor
        color_sensor = hardwareMap.get(ColorSensor.class, Config.REV_COLOR_RANGE);
        distance_sensor = hardwareMap.get(DistanceSensor.class, Config.REV_COLOR_RANGE);
    }


    private int cameraMonitorViewId = -1;

    private VuforiaLocalizer.Parameters vuforia_parameters = null;
    private VuforiaLocalizer vuforia = null;
    private VuforiaTrackables relicTrackables = null;
    private VuforiaTrackable relicTemplate = null;

    public void vuforia_init()
    {
        telemetry.addData("Status", "Initializing VuForia.");

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        vuforia_parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vuforia_parameters.vuforiaLicenseKey = Config.VUFORIA_LICENSE_KEY;
        vuforia_parameters.cameraDirection = Config.CAMERA_DIRECTION;
        vuforia = ClassFactory.createVuforiaLocalizer(vuforia_parameters);
        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        relicTrackables.activate();
    }


    private void run_to_position(DcMotor motor, int tgt, double power, int tolerance)
    {
        motor.setPower(0);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motor.setTargetPosition(tgt);

        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(power);

        while ( tolerance < Math.abs(motor.getTargetPosition()  - motor.getCurrentPosition()) ) {
            ;
        }

        sleep(Config.MOTOR_LAG);

        motor.setPower(0);
    }

    private void run_to_position(int fl_tgt, int fr_tgt, int br_tgt, int bl_tgt, double power, int tolerance)
    {
        port_bow_motor.setPower(0);
        stbd_bow_motor.setPower(0);
        stbd_aft_motor.setPower(0);
        port_aft_motor.setPower(0);

        port_bow_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stbd_bow_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        port_aft_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stbd_aft_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        port_bow_motor.setTargetPosition(fl_tgt);
        stbd_bow_motor.setTargetPosition(fr_tgt);
        port_aft_motor.setTargetPosition(bl_tgt);
        stbd_aft_motor.setTargetPosition(br_tgt);

        port_bow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stbd_bow_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        port_aft_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stbd_aft_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        port_bow_motor.setPower(power);
        stbd_bow_motor.setPower(power);
        stbd_aft_motor.setPower(power);
        port_aft_motor.setPower(power);

        while ( tolerance < Math.abs(port_bow_motor.getTargetPosition()  - port_bow_motor.getCurrentPosition())  ||
                tolerance < Math.abs(stbd_bow_motor.getTargetPosition() - stbd_bow_motor.getCurrentPosition()) ||
                tolerance < Math.abs(stbd_aft_motor.getTargetPosition()  - stbd_aft_motor.getCurrentPosition())  ||
                tolerance < Math.abs(port_aft_motor.getTargetPosition()   - port_aft_motor.getCurrentPosition())) {
            ;
        }

        sleep(Config.MOTOR_LAG);

        port_bow_motor.setPower(0);
        stbd_bow_motor.setPower(0);
        stbd_aft_motor.setPower(0);
        port_aft_motor.setPower(0);
    }
}