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
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
    // Declare OpMode members.
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_right  = null;
    private DcMotor back_left   = null;

    private DcMotor lift        = null;
    private DcMotor beam        = null;

    private DistanceSensor port_mr_range = null;
    private DistanceSensor stbd_mr_range = null;


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
    private static double TRANSITION_DELAY = 1;

    private static final int START      = 0;
    private static final int ROTATE     = START         + 1;
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
            "ROTATE", "MOTION", "CLAW", "TAIL", "BEAM", "ULTRASONIC", "COLOR", "IR", "VUFORIA", "IMU",
            "DONE"
    };

    private static final int BACK = 0;
    private static final int NEXT = 1;
    int[][] transition = {
            {DONE,          ROTATE},        // START
            {START,         MOTION},        // ROTATE
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
        } else if (state == ROTATE) {
            if (gamepad1.x && !gamepad1.b) {
                init_motors();

                front_left.setPower(0.1);
                front_right.setPower(0.1);
                back_right.setPower(0.1);
                back_left.setPower(0.1);
            } else if (!gamepad1.x && gamepad1.b) {
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
                front_right.setPower(0.1);
                back_right.setPower(-0.1);
                back_left.setPower(-0.1);
            } else if (!gamepad1.a && !gamepad1.b && gamepad1.x && !gamepad1.y) {
                // LEFT
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
        } else if (state == CLAW) {
            if (gamepad1.x && ! gamepad1.b) {           // open the claw
            } else if (! gamepad1.x && gamepad1.b) {    // close the claw
            }

            if (gamepad1.y && ! gamepad1.a) {           // raise the claw
                if (lift == null) {
                    lift = hardwareMap.get(DcMotor.class, Config.LIFT);
                    lift.setDirection(DcMotor.Direction.FORWARD);
                    lift.setPower(0);
                    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            } else if (! gamepad1.y && gamepad1.a) {    // lower the claw
                if (lift == null) {
                    lift = hardwareMap.get(DcMotor.class, Config.LIFT);
                    lift.setDirection(DcMotor.Direction.FORWARD);
                    lift.setPower(0);
                    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }
        } else if (state == TAIL) {
        } else if (state == BEAM) {
        } else if (state == ULTRASONIC) {
            if (gamepad1.x && port_mr_range == null) {
                port_mr_range = hardwareMap.get(DistanceSensor.class, Config.PORT_MR_RANGE);
            }

            if (gamepad1.b && stbd_mr_range == null) {
                stbd_mr_range = hardwareMap.get(DistanceSensor.class, Config.STBD_MR_RANGE);
            }

            if (gamepad1.x) {
                telemetry.addData("Port      MR Range", "in=%6.2f",
                        port_mr_range.getDistance(DistanceUnit.INCH));
            }

            if (gamepad1.b) {
                telemetry.addData("Starboard MR Range", "in=%6.2f",
                        stbd_mr_range.getDistance(DistanceUnit.INCH));
            }
        } else if (state == COLOR) {
        } else if (state == IR) {
        } else if (state == VUFORIA) {
        } else if (state == IMU) {
        } else if (state == DONE) {
        }

        // Show the elapsed game time.
        telemetry.addData("State", state_names[state]);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    private void init_motors() {
        if (front_left == null) {
            front_left = hardwareMap.get(DcMotor.class, Config.FRONT_LEFT);
            front_left.setDirection(DcMotor.Direction.FORWARD);
            front_left.setPower(0);
            front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (front_right == null) {
            front_right = hardwareMap.get(DcMotor.class, Config.FRONT_RIGHT);
            front_right.setDirection(DcMotor.Direction.FORWARD);
            front_right.setPower(0);
            front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            front_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (back_right == null) {
            back_right = hardwareMap.get(DcMotor.class, Config.BACK_RIGHT);
            back_right.setDirection(DcMotor.Direction.FORWARD);
            back_right.setPower(0);
            back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            back_right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (back_left == null) {
            back_left = hardwareMap.get(DcMotor.class, Config.BACK_LEFT);
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
