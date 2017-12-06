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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Test Holonomic Linear", group="Autonomous")
// @Disabled
public class TestHoloAuto_Linear extends LinearOpMode {
    // Declare OpMode members.
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_right  = null;
    private DcMotor back_left   = null;

    private AnalogInput rs0  = null;
    private Distance    rs0d = new Distance(10.616758844230123, -2.625694922444332, 5.292315651154265);
    private AnalogInput rs1  = null;
    private Distance    rs1d = new Distance(10.616758844230123, -2.625694922444332, 5.292315651154265);

    private ElapsedTime runtime = new ElapsedTime();

    private static final int fl_target_0 = -3500;
    private static final int fr_target_0 =  3500;
    private static final int br_target_0 =  3500;
    private static final int bl_target_0 = -3500;
    private static final double pwr_target_0 = 0.25;

    private static final int fl_home = 0;
    private static final int fr_home = 0;
    private static final int br_home = 0;
    private static final int bl_home = 0;

    private static final int pos_min = 10;

    @Override
    public void runOpMode() {
        /*
         * Initialize the drive system variables.
         */
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

        telemetry.addData("Status", "Initializing Range Sensors.");
        rs0 = hardwareMap.get(AnalogInput.class, "range sensor 0");
        rs1 = hardwareMap.get(AnalogInput.class, "range sensor 1");

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Status", "Initialization Complete.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        // Step through each leg of the path,
//        while (opModeIsActive()) {
//        }

        // go to first point
        telemetry.addData("Status", "Moving to point 0");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        front_left.setTargetPosition(fl_target_0);
        front_right.setTargetPosition(fr_target_0);
        back_right.setTargetPosition(br_target_0);
        back_left.setTargetPosition(bl_target_0);

        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left .setPower(pwr_target_0);
        front_right.setPower(pwr_target_0);
        back_left  .setPower(pwr_target_0);
        back_right .setPower(pwr_target_0);

        telemetry.addData("Status", "Waiting to arrive.");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        while ( pos_min < Math.abs(front_left .getCurrentPosition()-front_left .getTargetPosition()) &&
                pos_min < Math.abs(front_right.getCurrentPosition()-front_right.getTargetPosition()) &&
                pos_min < Math.abs(back_right .getCurrentPosition()-back_right .getTargetPosition()) &&
                pos_min < Math.abs(back_left  .getCurrentPosition()-back_left  .getTargetPosition())) {
            ;
        }


            // pause 5 seconds
        telemetry.addData("Status", "Sleeping");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        sleep(5000);

        // return to base
        telemetry.addData("Status", "Returning to home");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        front_left .setPower(0);
        front_right.setPower(0);
        back_left  .setPower(0);
        back_right .setPower(0);

        front_left.setTargetPosition(fl_home);
        front_right.setTargetPosition(fr_home);
        back_right.setTargetPosition(br_home);
        back_left.setTargetPosition(bl_home);

        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left .setPower(pwr_target_0);
        front_right.setPower(pwr_target_0);
        back_left  .setPower(pwr_target_0);
        back_right .setPower(pwr_target_0);

        telemetry.addData("Status", "Waiting to arrive at home.");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();

        while ( pos_min < Math.abs(front_left .getCurrentPosition()-front_left .getTargetPosition()) &&
                pos_min < Math.abs(front_right.getCurrentPosition()-front_right.getTargetPosition()) &&
                pos_min < Math.abs(back_right .getCurrentPosition()-back_right .getTargetPosition()) &&
                pos_min < Math.abs(back_left  .getCurrentPosition()-back_left  .getTargetPosition())) {
            ;
        }
        telemetry.addData("Status", "Arrived at home");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
        sleep(1000);

        telemetry.addData("Path", "Complete");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }
}
