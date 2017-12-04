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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="Rev Holonomic Autonomous", group="Autonomous")
// @Disabled
public class RevHolonomicAuto extends OpMode {
    private DcMotor front_left  = null;
    private DcMotor front_right = null;
    private DcMotor back_right  = null;
    private DcMotor back_left   = null;

    AnalogInput rs0  = null;
    Distance    rs0d = new Distance(10.616758844230123, -2.625694922444332, 5.292315651154265);
    AnalogInput rs1  = null;
    Distance    rs1d = new Distance(10.616758844230123, -2.625694922444332, 5.292315651154265);

    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
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

    public static final int start       = 0;
    public static final int initial_leg = 1;
    public static final int pause       = 2;
    public static final int return_leg  = 3;
    public static final int done        = 4;

    public static final int fl_stop_0  = -3500;
    public static final int fr_stop_0  =  3500;
    public static final int br_stop_0  =  3500;
    public static final int bl_stop_0  = -3500;

    int state = start;

    public static final double pause_time = 5;
    double timer_start = 0;

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Step through each leg of the path,

        if (state == start) {
            front_left.setTargetPosition(fl_stop_0);
            front_right.setTargetPosition(fr_stop_0);
            back_right.setTargetPosition(br_stop_0);
            back_left.setTargetPosition(bl_stop_0);

            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            front_left.setPower(0.1);
            front_right.setPower(0.1);
            back_left.setPower(0.1);
            back_right.setPower(0.1);

            state = initial_leg;
//            state = start;
        }

        if (state == initial_leg &&
                Math.abs(front_left .getCurrentPosition()-front_left .getTargetPosition()) < 10 &&
                Math.abs(front_right.getCurrentPosition()-front_right.getTargetPosition()) < 10 &&
                Math.abs(back_right .getCurrentPosition()-back_right .getTargetPosition()) < 10 &&
                Math.abs(back_left  .getCurrentPosition()-back_left  .getTargetPosition()) < 10) {

            front_left .setPower(0);
            front_right.setPower(0);
            back_left  .setPower(0);
            back_right .setPower(0);

            state = pause;
            timer_start = runtime.seconds();
        }

        if (state == pause && pause_time < (runtime.seconds() - timer_start)) {
            front_left .setTargetPosition(0);
            front_right.setTargetPosition(0);
            back_right .setTargetPosition(0);
            back_left  .setTargetPosition(0);

            front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            front_left.setPower(0.1);
            front_right.setPower(0.1);
            back_left.setPower(0.1);
            back_right.setPower(0.1);

            state = return_leg;
        }

        if (state == return_leg &&
                Math.abs(front_left .getCurrentPosition()-front_left .getTargetPosition()) < 10 &&
                Math.abs(front_right.getCurrentPosition()-front_right.getTargetPosition()) < 10 &&
                Math.abs(back_left  .getCurrentPosition()-back_left  .getTargetPosition()) < 10 &&
                Math.abs(back_right .getCurrentPosition()-back_right .getTargetPosition()) < 10) {
            state = done;
        }

            /*/
        if (10 < Math.abs(front_left .getCurrentPosition()-front_left .getTargetPosition()) ||
            10 < Math.abs(front_right.getCurrentPosition()-front_right.getTargetPosition()) ||
            10 < Math.abs(back_left  .getCurrentPosition()-back_left  .getTargetPosition()) ||
            10 < Math.abs(back_right .getCurrentPosition()-back_right .getTargetPosition())) {

            front_left .setPower(0);
            front_right.setPower(0);
            back_left  .setPower(0);
            back_right .setPower(0);

            front_left .setMode(DcMotor.RunMode.RUN_TO_POSITION);
            front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_left  .setMode(DcMotor.RunMode.RUN_TO_POSITION);
            back_right .setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
            /*/

        telemetry.addData("State", "%d", state);

            /*/
        telemetry.addData("Range", "%5.2fv %7.4fv %7.2f\"", rs0.getMaxVoltage(), rs0.getVoltage(),
                rs0d.distance(rs0.getVoltage()));

        telemetry.addData("Range", "%5.2fv %7.4fv %7.2f\"", rs1.getMaxVoltage(), rs1.getVoltage(),
                rs1d.distance(rs1.getVoltage()));
            /*/

        telemetry.addData("Motor Pos.", "%05d %05d %05d %05d",
                front_left.getCurrentPosition(), front_right.getCurrentPosition(),
                back_left .getCurrentPosition(), back_right .getCurrentPosition());

        telemetry.addData("Motor Tgt.", "%05d %05d %05d %05d",
                front_left.getTargetPosition(), front_right.getTargetPosition(),
                back_left .getTargetPosition(), back_right .getTargetPosition());

        telemetry.addData("Motor Pwr.", "%5.2f %5.2f %5.2f %5.2f",
                front_left.getPower(), front_right.getPower(),
                back_left .getPower(), back_right .getPower());

        telemetry.addData("Motor Mode",
                front_left.getMode() + " " + front_right.getMode() + "  " +
                back_left .getMode() + " " + back_right .getMode());

        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
