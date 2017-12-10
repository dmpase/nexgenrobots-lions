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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Linear EchoOpMode", group="Linear Opmode")
@Disabled
public class EchoOpMode_Linear extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Starting Initialization.");
        telemetry.addData("Status", "Initialization Completed.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double g1_lsx = gamepad1.left_stick_x;
            double g1_lsy = gamepad1.left_stick_y;
            double g1_ltr = gamepad1.left_trigger;
            double g1_rsx = gamepad1.right_stick_x;
            double g1_rsy = gamepad1.right_stick_y;
            double g1_rtr = gamepad1.right_trigger;

            String gp1_buttons =
                    gamepad1.id + " " +
                            (gamepad1.a?"A":"") +
                            (gamepad1.b?"B":"") +
                            (gamepad1.x?"X":"") +
                            (gamepad1.y?"Y":"") +
                            (gamepad1.dpad_up?"U":"") +
                            (gamepad1.dpad_down?"D":"") +
                            (gamepad1.dpad_left?"L":"") +
                            (gamepad1.dpad_right?"R":"") +
                            (gamepad1.left_bumper?"{":"") +
                            (gamepad1.right_bumper?"}":"") +
                            (gamepad1.guide?"g":"") +
                            (gamepad1.left_stick_button?"[":"") +
                            (gamepad1.right_stick_button?"]":"") +
                            (gamepad1.back?"<":"") +
                            (gamepad1.start?">":"") +
                            ""
                    ;


            double g2_lsx = gamepad2.left_stick_x;
            double g2_lsy = gamepad2.left_stick_y;
            double g2_ltr = gamepad2.left_trigger;
            double g2_rsx = gamepad2.right_stick_x;
            double g2_rsy = gamepad2.right_stick_y;
            double g2_rtr = gamepad2.right_trigger;

            String gp2_buttons =
                    gamepad2.id + " " +
                            (gamepad2.a?"A":"") +
                            (gamepad2.b?"B":"") +
                            (gamepad2.x?"X":"") +
                            (gamepad2.y?"Y":"") +
                            (gamepad2.dpad_up?"U":"") +
                            (gamepad2.dpad_down?"D":"") +
                            (gamepad2.dpad_left?"L":"") +
                            (gamepad2.dpad_right?"R":"") +
                            (gamepad2.left_bumper?"{":"") +
                            (gamepad2.right_bumper?"}":"") +
                            (gamepad2.guide?"g":"") +
                            (gamepad2.left_stick_button?"[":"") +
                            (gamepad2.right_stick_button?"]":"") +
                            (gamepad2.back?"<":"") +
                            (gamepad2.start?">":"") +
                            ""
                    ;


            // Show the elapsed game time and wheel power.
            telemetry.addData("1", "L:(%.2f, %.2f, %.2f) R:(%.2f, %.2f, %.2f)", g1_lsx, g1_lsy, g1_ltr, g1_rsx, g1_rsy, g1_rtr);
            telemetry.addData("", gp1_buttons);
            telemetry.addData("2", "L:(%.2f, %.2f, %.2f) R:(%.2f, %.2f, %.2f)", g2_lsx, g2_lsy, g2_ltr, g2_rsx, g2_rsy, g2_rtr);
            telemetry.addData("", gp2_buttons);
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.update();
        }
    }
}
