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
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * Use Android Studio to copy this class, and paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

// @TeleOp(name="Diagnostics", group="Iterative Opmode")
// @Disabled
public abstract class NgDiagnostics extends OpMode {

    public NgDrive   drive = null;
    public NgClaw    claw  = null;
    public NgTail    tail  = null;
    public NgBeam    beam  = null;
    public NgUltra   ultra = null;
    public NgIR      ir    = null;
    public NgVuforia vufo  = null;

    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    public abstract void init();

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
    private static double TRANSITION_DELAY = 0.5;

    private static final int START      = 0;
    private static final int GAMEPAD    = START         + 1;
    private static final int ROTATE     = GAMEPAD       + 1;
    private static final int MOTION     = ROTATE        + 1;
    private static final int CLAW       = MOTION        + 1;
    private static final int TAIL       = CLAW          + 1;
    private static final int BEAM       = TAIL          + 1;
    private static final int ULTRASONIC = BEAM          + 1;
    private static final int IR         = ULTRASONIC    + 1;
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
            {BEAM,          IR},            // ULTRASONIC
            {ULTRASONIC,    VUFORIA},       // IR
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
            telemetry.addData(state_names[state], "%8.0fs", runtime.seconds());
        } else if (state == GAMEPAD) {
            telemetry.addData(state_names[state], "%8.0fs", runtime.seconds());
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
            telemetry.addData(state_names[state], "X/B to turn left/right.");
            if ((gamepad1.x) && !(gamepad1.b)) {
                drive.init();
                drive.turn(0.1);
            } else if (!(gamepad1.x || gamepad1.left_bumper) && (gamepad1.b || gamepad1.right_bumper)) {
                drive.init();
                drive.turn(-0.1);
            } else {
                drive.init();
                drive.turn(0.0);
            }
        } else if (state == MOTION) {
            telemetry.addData(state_names[state], "Y/B to go forward/backward.");
            telemetry.addData(state_names[state], "X/B to go left/right.");
            if (!gamepad1.a && !gamepad1.b && !gamepad1.x && gamepad1.y) {
                // FORWARD
                drive.init();
                drive.move(0,0,0.1);
            } else if (!gamepad1.a && gamepad1.b && !gamepad1.x && !gamepad1.y) {
                // RIGHT
                drive.init();
                drive.move(270,0,0.1);
            } else if (gamepad1.a && !gamepad1.b && !gamepad1.x && !gamepad1.y) {
                // BACKWARD
                drive.init();
                drive.move(180,0,0.1);
            } else if (!gamepad1.a && !gamepad1.b && gamepad1.x && !gamepad1.y) {
                // LEFT
                drive.init();
                drive.move(90,0,0.1);
            } else {
                drive.init();
                drive.move(0,0,0);
            }
        } else if (state == CLAW) {
            telemetry.addData(state_names[state], "B/X to open/close claw.");
            telemetry.addData(state_names[state], "Y/A to raise/lower claw.");
            if (gamepad1.x && ! gamepad1.b) {           // open the claw
                claw.init();
                claw.open();
            } else if (! gamepad1.x && gamepad1.b) {    // close the claw
                claw.init();
                claw.open();
            }

            if (gamepad1.y && ! gamepad1.a) {           // raise the claw
                claw.init();
                claw.raise(1.0);
            } else if (! gamepad1.y && gamepad1.a) {    // lower the claw
                claw.init();
                claw.raise(0.0);
            }
        } else if (state == TAIL) {
            telemetry.addData(state_names[state], "Y/A to raise/lower tail.");
            telemetry.addData(state_names[state], "X to read the color.");
            telemetry.addData(state_names[state], "B to read the distance.");
            if (gamepad1.y && ! gamepad1.a) {           // raise the tail
                tail.init();
                tail.raise(1.0);
            } else if (! gamepad1.y && gamepad1.a) {    // lower the tail
                tail.init();
                tail.raise(0.0);
            } else if (gamepad1.x) {                    // read the color sensor
                tail.init();
                double[] color = tail.color();
                telemetry.addData(state_names[state], "a=%.2f r=%.2f g=%.2f b=%.2f",
                        color[NgTail.ALPHA], color[NgTail.RED ],
                        color[NgTail.GREEN], color[NgTail.BLUE]);
            } else if (gamepad1.b) {                    // read the distance sensor
                tail.init();
                double distance = tail.distance();
                telemetry.addData(state_names[state], "%.2f", distance);
            }
        } else if (state == BEAM) {
            telemetry.addData(state_names[state], "DPAD L/R to extend/retract beam.");
            telemetry.addData(state_names[state], "DPAD U/D to raise/lower swivel.");
            telemetry.addData(state_names[state], "Bumper L/R to close/open claw.");
            if (gamepad1.dpad_left && ! gamepad1.dpad_right) {              // extend the beam
                beam.init();
                beam.extend(1.0);
            } else if (! gamepad1.dpad_left && gamepad1.dpad_right) {       // retract the beam
                beam.init();
                beam.extend(0.0);
            } else if (gamepad1.dpad_up && ! gamepad1.dpad_down) {          // swivel beam claw up
                beam.init();
                beam.raise();
            } else if ( ! gamepad1.dpad_up && gamepad1.dpad_down) {         // swivel beam claw down
                beam.init();
                beam.lower();
            } else if (gamepad1.left_bumper && ! gamepad1.right_bumper)  {  // open beam claw
                beam.init();
                beam.open();
            } else if ( ! gamepad1.left_bumper && gamepad1.right_bumper) {  // close beam claw
                beam.init();
                beam.close();
            }
        } else if (state == ULTRASONIC) {
            telemetry.addData(state_names[state], "A/B/X/Y to show.");
            if (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y) {
                ultra.init();
                double[] distances = ultra.read();
                telemetry.addData(state_names[state], "port bow=%6.2f stbd bow=%6.2f",
                        distances[NgUltra.PORT_BOW], distances[NgUltra.STBD_BOW]);
                telemetry.addData(state_names[state], "port aft=%6.2f stbd aft=%6.2f",
                        distances[NgUltra.PORT_AFT], distances[NgUltra.STBD_AFT]);
            }
        } else if (state == IR) {
            telemetry.addData(state_names[state], "A/B/X/Y to show.");
            if (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y) {
                ir.init();
                double[] distances = ir.read();
                telemetry.addData(state_names[state], "port bow=%6.2f stbd bow=%6.2f",
                        distances[NgIR.PORT_BOW], distances[NgIR.STBD_BOW]);
                telemetry.addData(state_names[state], "port aft=%6.2f stbd aft=%6.2f",
                        distances[NgIR.PORT_AFT], distances[NgIR.STBD_AFT]);
            }
        } else if (state == VUFORIA) {
            telemetry.addData(state_names[state], "A/B/X/Y to show.");
            if (gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y) {
                vufo.init();
                int vr = vufo.read();
                if (vr == NgVuforia.LEFT) {
                    telemetry.addData(state_names[state], "LEFT.");
                } else if (vr == NgVuforia.CENTER) {
                    telemetry.addData(state_names[state], "CENTER.");
                } else if (vr == NgVuforia.RIGHT) {
                    telemetry.addData(state_names[state], "RIGHT.");
                }
            }
        } else if (state == IMU) {
            telemetry.addData(state_names[state], "%8.0fs", runtime.seconds());
        } else if (state == DONE) {
            telemetry.addData(state_names[state], "%8.0fs", runtime.seconds());
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
    public void stop()
    {
        drive.stop();
        claw.stop();
        tail.stop();
        beam.stop();
        ultra.stop();
        ir.stop();
        vufo.stop();
    }
}
