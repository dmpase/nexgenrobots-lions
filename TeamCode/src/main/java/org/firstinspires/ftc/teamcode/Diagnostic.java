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
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Diagnostic", group="Iterative Opmode")
// @Disabled
public class Diagnostic extends OpMode {
    // locomotion motors
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_right  = null;
    private DcMotor back_left   = null;

    // claw and tail servos
    private Servo port_claw = null;
    private Servo stbd_claw = null;
    private Servo tail      = null;

    // claw lift and beam motors
    private DcMotor lift        = null;
    private DcMotor beam        = null;

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
    Distance    ir_v2in = new Distance(10.616758844230123, -2.625694922444332, 5.292315651154265);

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
    public void init() {
        telemetry.addData("Status", "Starting Initialization.");

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


    private double transition_start = 0;
    private static double TRANSITION_DELAY = 0.75;

    private static final int START      = 0;
    private static final int GAMEPAD    = START         + 1;
    private static final int ROTATE     = GAMEPAD       + 1;
    private static final int MOTION     = ROTATE        + 1;
    private static final int CLAW       = MOTION        + 1;
    private static final int TAIL       = CLAW          + 1;
    private static final int BEAM       = TAIL          + 1;
    private static final int ULTRASONIC = BEAM          + 1;
    private static final int COLOR      = ULTRASONIC    + 1;
    private static final int IR         = COLOR         + 1;
    private static final int VUFORIA    = IR            + 1;
    private static final int IMU        = VUFORIA       + 1;
    private static final int DONE       = IMU           + 1;
    private int state = START;

    private static final String[] state_names = {
            "START",
            "GAMEPAD", "ROTATE", "MOTION", "CLAW", "TAIL", "BEAM", "ULTRASONIC", "COLOR", "IR", "VUFORIA", "IMU",
            "DONE"
    };

    private static final int BACK = 0;
    private static final int NEXT = 1;
    int[][] transition = {
            {DONE,          GAMEPAD},       // START
            {START,         ROTATE},        // GAMEPAD
            {GAMEPAD,       MOTION},        // ROTATE
            {ROTATE,        CLAW},          // MOTION
            {MOTION,        TAIL},          // CLAW
            {CLAW,          BEAM},          // TAIL
            {TAIL,          ULTRASONIC},    // BEAM
            {BEAM,          COLOR},         // ULTRASONIC
            {ULTRASONIC,    IR},            // COLOR
            {COLOR,         VUFORIA},       // IR
            {IR,            IMU},           // VUFORIA
            {VUFORIA,       DONE},          // IMU
            {IMU,           START},         // DONE
    };

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if (gamepad1.start && TRANSITION_DELAY < (runtime.seconds() - transition_start)) {
            state = transition[state][NEXT];
            transition_start = runtime.seconds();
        } else if (gamepad1.back && TRANSITION_DELAY < (runtime.seconds() - transition_start)) {
            state = transition[state][BACK];
            transition_start = runtime.seconds();
        }

        if (state == START) {
            ;
        } else if (state == GAMEPAD) {
            if (gamepad1.a) {
                telemetry.addData("Gamepad 1", "A");
            } else if (gamepad1.b) {
                telemetry.addData("Gamepad 1", "B");
            } else if (gamepad1.x) {
                telemetry.addData("Gamepad 1", "X");
            } else if (gamepad1.y) {
                telemetry.addData("Gamepad 1", "Y");
            } else if (gamepad1.dpad_down) {
                telemetry.addData("Gamepad 1", "DPAD Down");
            } else if (gamepad1.dpad_left) {
                telemetry.addData("Gamepad 1", "DPAD Left");
            } else if (gamepad1.dpad_right) {
                telemetry.addData("Gamepad 1", "DPAD Right");
            } else if (gamepad1.dpad_up) {
                telemetry.addData("Gamepad 1", "DPAD Up");
            } else if (gamepad1.dpad_down) {
                telemetry.addData("Gamepad 1", "DPAD Down");
            } else if (gamepad1.left_bumper) {
                telemetry.addData("Gamepad 1", "Left Bumper");
            } else if (gamepad1.right_bumper) {
                telemetry.addData("Gamepad 1", "Right Bumper");
            } else if (gamepad1.left_stick_button) {
                telemetry.addData("Gamepad 1", "Left Stick Button");
            } else if (gamepad1.right_stick_button) {
                telemetry.addData("Gamepad 1", "Right Stick Button");
            } else if (gamepad1.guide) {
                telemetry.addData("Gamepad 1", "Guide");
            } else {
                telemetry.addData("Gamepad 1", "(none)");
            }

            telemetry.addData("Gamepad 1", "[%5.2f %5.2f] [%5.2f %5.2f] [%5.2f %5.2f]",
                    gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.right_stick_x, gamepad1.right_stick_y,
                    gamepad1.left_trigger, gamepad1.right_trigger);

            if (gamepad2.a) {
                telemetry.addData("Gamepad 2", "A");
            } else if (gamepad2.b) {
                telemetry.addData("Gamepad 2", "B");
            } else if (gamepad2.x) {
                telemetry.addData("Gamepad 2", "X");
            } else if (gamepad2.y) {
                telemetry.addData("Gamepad 2", "Y");
            } else if (gamepad2.dpad_down) {
                telemetry.addData("Gamepad 2", "DPAD Down");
            } else if (gamepad2.dpad_left) {
                telemetry.addData("Gamepad 2", "DPAD Left");
            } else if (gamepad2.dpad_right) {
                telemetry.addData("Gamepad 2", "DPAD Right");
            } else if (gamepad2.dpad_up) {
                telemetry.addData("Gamepad 2", "DPAD Up");
            } else if (gamepad2.dpad_down) {
                telemetry.addData("Gamepad 2", "DPAD Down");
            } else if (gamepad2.left_bumper) {
                telemetry.addData("Gamepad 2", "Left Bumper");
            } else if (gamepad2.right_bumper) {
                telemetry.addData("Gamepad 2", "Right Bumper");
            } else if (gamepad2.left_stick_button) {
                telemetry.addData("Gamepad 2", "Left Stick Button");
            } else if (gamepad2.right_stick_button) {
                telemetry.addData("Gamepad 2", "Right Stick Button");
            } else if (gamepad2.guide) {
                telemetry.addData("Gamepad 2", "Guide");
            } else {
                telemetry.addData("Gamepad 2", "(none)");
            }

            telemetry.addData("Gamepad 2", "[%5.2f %5.2f] [%5.2f %5.2f] [%5.2f %5.2f]",
                    gamepad2.left_stick_x,  gamepad2.left_stick_y,
                    gamepad2.right_stick_x, gamepad2.right_stick_y,
                    gamepad2.left_trigger,  gamepad2.right_trigger);
        } else if (state == ROTATE) {
            if ((gamepad1.x || gamepad1.left_bumper) && !(gamepad1.b || gamepad1.right_bumper)) {
                init_motors();

                front_left.setPower(0.1);
                front_right.setPower(0.1);
                back_right.setPower(0.1);
                back_left.setPower(0.1);
            } else if (!(gamepad1.x || gamepad1.left_bumper) && (gamepad1.b || gamepad1.right_bumper)) {
                init_motors();

                front_left.setPower(-0.1);
                front_right.setPower(-0.1);
                back_right.setPower(-0.1);
                back_left.setPower(-0.1);
            } else {
                if (front_left != null) {
                    front_left.setPower(0);
                }

                if (front_right != null) {
                    front_right.setPower(0);
                }

                if (back_right != null) {
                    back_right.setPower(0);
                }

                if (back_left != null) {
                    back_left.setPower(0);
                }
            }
        } else if (state == MOTION) {
            if (!gamepad1.a && !gamepad1.b && !gamepad1.x && gamepad1.y) {
                // FORWARD
                init_motors();

                front_left.setPower(-0.1);
                front_right.setPower(0.1);
                back_right.setPower(0.1);
                back_left.setPower(-0.1);
            } else if (!gamepad1.a && gamepad1.b && !gamepad1.x && !gamepad1.y) {
                // RIGHT
                init_motors();

                front_left.setPower(-0.1);
                front_right.setPower(-0.1);
                back_right.setPower(0.1);
                back_left.setPower(0.1);
            } else if (gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y) {
                // BACKWARD
                init_motors();

                front_left.setPower(0.1);
                front_right.setPower(-0.1);
                back_right.setPower(-0.1);
                back_left.setPower(0.1);
            } else if (!gamepad1.a && !gamepad1.b && gamepad1.x && !gamepad1.y) {
                // LEFT
                init_motors();

                front_left.setPower(0.1);
                front_right.setPower(0.1);
                back_right.setPower(-0.1);
                back_left.setPower(-0.1);
            } else {
                if (front_left != null) {
                    front_left.setPower(0);
                }

                if (front_right != null) {
                    front_right.setPower(0);
                }

                if (back_right != null) {
                    back_right.setPower(0);
                }

                if (back_left != null) {
                    back_left.setPower(0);
                }
            }
        } else if (state == CLAW) {
            if (gamepad1.x && ! gamepad1.b) {           // open the claw
                if (port_claw == null) {
                    port_claw = hardwareMap.get(Servo.class, Config.PORT_CLAW);
                    port_claw.setDirection(Servo.Direction.FORWARD);
                }

                if (stbd_claw == null) {
                    stbd_claw = hardwareMap.get(Servo.class, Config.STBD_CLAW);
                    stbd_claw.setDirection(Servo.Direction.FORWARD);
                }

                port_claw.setPosition(Config.PORT_CLAW_OPENED);
                stbd_claw.setPosition(Config.STBD_CLAW_OPENED);
            } else if (! gamepad1.x && gamepad1.b) {    // close the claw
                if (port_claw == null) {
                    port_claw = hardwareMap.get(Servo.class, Config.PORT_CLAW);
                    port_claw.setDirection(Servo.Direction.FORWARD);
                }

                if (stbd_claw == null) {
                    stbd_claw = hardwareMap.get(Servo.class, Config.STBD_CLAW);
                    stbd_claw.setDirection(Servo.Direction.FORWARD);
                }

                port_claw.setPosition(Config.PORT_CLAW_CLOSED);
                stbd_claw.setPosition(Config.STBD_CLAW_CLOSED);
            }

            if (gamepad1.y && ! gamepad1.a) {           // raise the claw
                if (lift == null) {
                    lift = hardwareMap.get(DcMotor.class, Config.LIFT);
                    lift.setDirection(Config.LIFT_DIRECTION);
                    lift.setPower(0);
                    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                lift.setPower(0);
                lift.setTargetPosition(Config.LIFT_TARGET_HI);
                lift.setPower(Config.LIFT_POWER);

                while ( Config.MOTOR_TARGET_TOLERANCE < Math.abs(lift.getTargetPosition() - lift.getCurrentPosition()) ) {
                    ;
                }

                lift.setPower(0);
            } else if (! gamepad1.y && gamepad1.a) {    // lower the claw
                if (lift == null) {
                    lift = hardwareMap.get(DcMotor.class, Config.LIFT);
                    lift.setDirection(Config.LIFT_DIRECTION);
                    lift.setPower(0);
                    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                lift.setPower(0);
                lift.setTargetPosition(Config.LIFT_TARGET_LO);
                lift.setPower(Config.LIFT_POWER);

                while ( Config.MOTOR_TARGET_TOLERANCE < Math.abs(lift.getTargetPosition() - lift.getCurrentPosition()) ) {
                    ;
                }

                lift.setPower(0);
            }

            if (gamepad1.a || gamepad1.y|| gamepad1.right_bumper || gamepad1.left_bumper) {
                telemetry.addData("Lift", "%4d", lift.getCurrentPosition());
            }
        } else if (state == TAIL) {
            if (gamepad1.y && ! gamepad1.a) {           // raise the tail
                if (tail == null) {
                    tail  = hardwareMap.get(Servo.class, Config.TAIL);
                    tail.setDirection(Servo.Direction.FORWARD);
                }

                tail.setPosition(Config.TAIL_POS_UP);
            } else if (! gamepad1.y && gamepad1.a) {    // lower the tail
                if (tail == null) {
                    tail  = hardwareMap.get(Servo.class, Config.TAIL);
                    tail.setDirection(Servo.Direction.FORWARD);
                }

                tail.setPosition(Config.TAIL_POS_DN);
            }
        } else if (state == BEAM) {
            if (gamepad1.x && ! gamepad1.b) {           // extend the beam
                if (beam == null) {
                    beam = hardwareMap.get(DcMotor.class, Config.BEAM);
                    beam.setDirection(DcMotor.Direction.FORWARD);
                    beam.setPower(0);
                    beam.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    beam.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                beam.setPower(0);
                beam.setTargetPosition(Config.BEAM_TARGET_OUT);
                beam.setPower(Config.BEAM_POWER);

                while ( Config.MOTOR_TARGET_TOLERANCE < Math.abs(beam.getTargetPosition() - beam.getCurrentPosition()) ) {
                    ;
                }

                beam.setPower(0);
            } else if (! gamepad1.x && gamepad1.b) {    // retract the beam
                if (beam == null) {
                    beam = hardwareMap.get(DcMotor.class, Config.BEAM);
                    beam.setDirection(Config.BEAM_DIRECTION);
                    beam.setPower(0);
                    beam.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    beam.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                beam.setPower(0);
                beam.setTargetPosition(Config.BEAM_TARGET_IN);
                beam.setPower(Config.BEAM_POWER);

                while ( Config.MOTOR_TARGET_TOLERANCE < Math.abs(beam.getTargetPosition() - beam.getCurrentPosition()) ) {
                    ;
                }

                beam.setPower(0);
            }
        } else if (state == ULTRASONIC) {
            if (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y) {
                if (port_mr_range == null) {
                    port_mr_range = hardwareMap.get(DistanceSensor.class, Config.PORT_MR_RANGE);
                }

                if (stbd_mr_range == null) {
                    stbd_mr_range = hardwareMap.get(DistanceSensor.class, Config.STBD_MR_RANGE);
                }

                telemetry.addData("MR Range", "[%6.2f\", %6.2f\"]",
                        port_mr_range.getDistance(DistanceUnit.INCH),
                        stbd_mr_range.getDistance(DistanceUnit.INCH));
            }
        } else if (state == COLOR) {
            if (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y) {
                if (color_sensor == null) {
                    color_sensor = hardwareMap.get(ColorSensor.class, Config.REV_COLOR_RANGE);
                    distance_sensor = hardwareMap.get(DistanceSensor.class, Config.REV_COLOR_RANGE);
                }

                telemetry.addData("Distance", "cm=%6.2f",
                        distance_sensor.getDistance(DistanceUnit.CM));

                telemetry.addData("Color", "a=%03d r=%03d g=%03d b=%03d %s",
                        color_sensor.alpha(),
                        color_sensor.red(), color_sensor.green(), color_sensor.blue(),
                        (color_sensor.red() < color_sensor.blue())?"BLUE":"RED");
            }
        } else if (state == IR) {
            if (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y) {
                if (port_ir_aft == null) {
                    port_ir_aft = hardwareMap.get(AnalogInput.class, Config.PORT_IR_AFT);
                }

                if (stbd_ir_aft == null) {
                    stbd_ir_aft = hardwareMap.get(AnalogInput.class, Config.STBD_IR_AFT);
                }

                if (port_ir_bow == null) {
                    port_ir_bow = hardwareMap.get(AnalogInput.class, Config.PORT_IR_BOW);
                }

                if (stbd_ir_bow == null) {
                    stbd_ir_bow = hardwareMap.get(AnalogInput.class, Config.STBD_IR_BOW);
                }

                telemetry.addData("Port",
                        "[%6.4f:%6.2f, %6.4f:%6.2f]",
                        port_ir_aft.getVoltage(), ir_v2in.distance(port_ir_aft.getVoltage()),
                        port_ir_bow.getVoltage(), ir_v2in.distance(port_ir_bow.getVoltage()));

                telemetry.addData("Starboard",
                        "[%6.4f:%6.2f, %6.4f:%6.2f]",
                        stbd_ir_aft.getVoltage(), ir_v2in.distance(stbd_ir_aft.getVoltage()),
                        stbd_ir_bow.getVoltage(), ir_v2in.distance(stbd_ir_bow.getVoltage()));
            }
        } else if (state == VUFORIA) {
            if (gamepad1.a || gamepad1.b) {
                if (cameraMonitorViewId < 0) {
                    cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                            "cameraMonitorViewId",
                            "id", hardwareMap.appContext.getPackageName());
                } else if (vuforia_parameters == null) {
                    vuforia_parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
                    vuforia_parameters.vuforiaLicenseKey = Config.VUFORIA_LICENSE_KEY;
                    vuforia_parameters.cameraDirection = Config.CAMERA_DIRECTION;
                } else if (vuforia == null) {
                    vuforia = ClassFactory.createVuforiaLocalizer(vuforia_parameters);
                } else if (relicTrackables == null) {
                    relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
                } else if (relicTemplate == null) {
                    relicTemplate = relicTrackables.get(0);
                    relicTemplate.setName("relicVuMarkTemplate");
                }
            }

            if (gamepad1.a && relicTrackables != null) {
                relicTrackables.activate();
            } else if (gamepad1.b && relicTrackables != null) {
                relicTrackables.deactivate();
            }

            if (relicTrackables != null) {
                vuMark = RelicRecoveryVuMark.from(relicTemplate);
                telemetry.addData("VuForia", "vuMark='%s'", vuMark.name());
            }
        } else if (state == IMU) {
        } else if (state == DONE) {
            ;
        }

        // Show the elapsed game time.
        telemetry.addData("State", "%10s %8.0f", state_names[state], runtime.seconds());
    }

    private void init_motors() {
        if (front_left == null) {
            front_left = hardwareMap.get(DcMotor.class, Config.PORT_BOW);
            front_left.setDirection(DcMotor.Direction.FORWARD);
            front_left.setPower(0);
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (front_right == null) {
            front_right = hardwareMap.get(DcMotor.class, Config.STBD_BOW);
            front_right.setDirection(DcMotor.Direction.FORWARD);
            front_right.setPower(0);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (back_right == null) {
            back_right = hardwareMap.get(DcMotor.class, Config.STBD_AFT);
            back_right.setDirection(DcMotor.Direction.FORWARD);
            back_right.setPower(0);
            back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (back_left == null) {
            back_left = hardwareMap.get(DcMotor.class, Config.PORT_AFT);
            back_left.setDirection(DcMotor.Direction.FORWARD);
            back_left.setPower(0);
            back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
