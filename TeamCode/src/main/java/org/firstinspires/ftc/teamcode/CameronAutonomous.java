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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
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

@Autonomous(name="Cameron Autonomous", group="Autonomous")
// @Disabled
public class CameronAutonomous extends LinearOpMode {
    // Declare OpMode members.

    public static enum Command {ROTATE, FORWARD, BACKWARD, LEFT, RIGHT, ADJUST, OPEN_CLAW, CLOSE_CLAW}

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

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        runtime.reset();

        // read port and starboard sensor to find quadrant (3rd quadrant; blue)

        double port_dist = rs2d.distance(prs_lo.getVoltage());
        double starboard_dist = rs2d.distance(srs_lo.getVoltage());

        final int UNKNOWN    = 0;
        final int BLUE_LEFT  = 1;
        final int BLUE_RIGHT = 2;
        final int RED_LEFT   = 3;
        final int RED_RIGHT  = 4;
        final String[] QUADRANT_NAME = {"UNKNOWN", "BLUE LEFT", "BLUE RIGHT", "RED LEFT", "RED RIGHT"};
        int quadrant = UNKNOWN;

        final double SHORT  = 36;
        final double MEDIUM = 48;
        final double LONG   = 60;

        if (starboard_dist < SHORT) {
            quadrant = BLUE_RIGHT;
        } else if (starboard_dist < MEDIUM) {
            quadrant = RED_RIGHT;
        } else if (port_dist < SHORT) {
            quadrant = RED_LEFT;
        } else if (port_dist < MEDIUM) {
            quadrant = BLUE_LEFT;
        }

        telemetry.addData("Quadrant", "%s(%d) %6.2f %6.2f %6.2f %6.2f",
                QUADRANT_NAME[quadrant], quadrant, port_dist, starboard_dist,
                rs2d.distance(prs_hi.getVoltage()), rs2d.distance(srs_hi.getVoltage()));
        telemetry.update();


        // vuforia

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);

        telemetry.addData("VuMark", "%s", vuMark.toString());
        //Vuforia reading: left, center, or right; left = -1, center = 0, right = 1;
        vuforiaresult = VUFORIA_CENTER;

        // lower tail
        // read color
        // if blue, clockwise then counter clockwise, opposite for red
        // raise tail

        // deliver the block to the crypto box

        Object[][] program = blue_right_cmd;

        for (int i=0; i < program.length; i++) {
            execute(program[i]);
        }

        // open claw

        telemetry.addData("Path", "Complete");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }

    private static final double AUTO_PWR = 0.1;
    private static final int    AUTO_TOL = 10;

    private static final Object[][] blue_left_cmd = {
            {Command.ROTATE,           90.0, AUTO_PWR, AUTO_TOL},
            {Command.FORWARD,          36.0, AUTO_PWR, AUTO_TOL},
            {Command.ROTATE,           90.0, AUTO_PWR, AUTO_TOL},
            {Command.ADJUST,              8, AUTO_PWR, AUTO_TOL},
            {Command.FORWARD,          18.0, AUTO_PWR, AUTO_TOL},
            {Command.OPEN_CLAW,                                },
            {Command.BACKWARD,          4.0, AUTO_PWR, AUTO_TOL},
            {Command.CLOSE_CLAW,                               },
    };

    private static final Object[][] blue_right_cmd = {
            {Command.ROTATE,           90.0, AUTO_PWR, AUTO_TOL},
            {Command.FORWARD,          36.0, AUTO_PWR, AUTO_TOL},
            {Command.RIGHT,            12.0, AUTO_PWR, AUTO_TOL},
            {Command.ADJUST,              8, AUTO_PWR, AUTO_TOL},
            {Command.FORWARD,          12.0, AUTO_PWR, AUTO_TOL},
            {Command.OPEN_CLAW,                                },
            {Command.BACKWARD,          4.0, AUTO_PWR, AUTO_TOL},
            {Command.CLOSE_CLAW,                               },
    };

    private static final Object[][] red_left_cmd = {
            {Command.ROTATE,          -90.0, AUTO_PWR, AUTO_TOL},
            {Command.FORWARD,          36.0, AUTO_PWR, AUTO_TOL},
            {Command.ROTATE,          -90.0, AUTO_PWR, AUTO_TOL},
            {Command.ADJUST,              8, AUTO_PWR, AUTO_TOL},
            {Command.FORWARD,          18.0, AUTO_PWR, AUTO_TOL},
            {Command.OPEN_CLAW,                                },
            {Command.BACKWARD,          4.0, AUTO_PWR, AUTO_TOL},
            {Command.CLOSE_CLAW,                               },
    };

    private static final Object[][] red_right_cmd = {
            {Command.ROTATE,          -90.0, AUTO_PWR, AUTO_TOL},
            {Command.FORWARD,          36.0, AUTO_PWR, AUTO_TOL},
            {Command.LEFT,             12.0, AUTO_PWR, AUTO_TOL},
            {Command.ADJUST,              8, AUTO_PWR, AUTO_TOL},
            {Command.FORWARD,          12.0, AUTO_PWR, AUTO_TOL},
            {Command.OPEN_CLAW,                                },
            {Command.BACKWARD,          4.0, AUTO_PWR, AUTO_TOL},
            {Command.CLOSE_CLAW,                               },
    };

    final int OPCODE    = 0;
    final int ANGLE     = 1;
    final int INCHES    = 1;
    final int POWER     = 2;
    final int TOLERANCE = 3;

    private static final int VUFORIA_LEFT = -1;
    private static final int VUFORIA_CENTER = 0;
    private static final int VUFORIA_RIGHT = 1;
    private int vuforiaresult = 0;

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
        } else if (op_code == Command.CLOSE_CLAW) {
        }
    }

    DcMotor front_left  = null;
    DcMotor front_right = null;
    DcMotor back_right  = null;
    DcMotor back_left   = null;

    public void motor_init()
    {
        telemetry.addData("Status", "Initializing Motors.");

        front_left  = hardwareMap.get(DcMotor.class, Config.PORT_BOW);
        front_right = hardwareMap.get(DcMotor.class, Config.STBD_BOW);
        back_right  = hardwareMap.get(DcMotor.class, Config.STBD_AFT);
        back_left   = hardwareMap.get(DcMotor.class, Config.PORT_AFT);

        front_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);

        front_left.setDirection(DcMotor.Direction.FORWARD);
        front_right.setDirection(DcMotor.Direction.FORWARD);
        back_right.setDirection(DcMotor.Direction.FORWARD);
        back_left.setDirection(DcMotor.Direction.FORWARD);

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void servo_init()
    {

    }


    AnalogInput prs_lo = null;
    AnalogInput srs_lo = null;
    AnalogInput prs_hi = null;
    AnalogInput srs_hi = null;
    Distance    rs2d   = new Distance(10.616758844230123, -2.625694922444332, 5.292315651154265);

    public void sensor_init()
    {
        telemetry.addData("Status", "Initializing Sensors.");

        // white wire is high, blue wire is low, ports 0,1 to starboard, ports 2,3 to port
        prs_lo = hardwareMap.get(AnalogInput.class, Config.PORT_IR_AFT);
        srs_lo = hardwareMap.get(AnalogInput.class, Config.STBD_IR_AFT);
        prs_hi = hardwareMap.get(AnalogInput.class, Config.PORT_IR_BOW);
        srs_hi = hardwareMap.get(AnalogInput.class, Config.STBD_IR_BOW);
    }


    int cameraMonitorViewId = -1;

    VuforiaLocalizer.Parameters vuforia_parameters = null;
    VuforiaLocalizer vuforia = null;
    VuforiaTrackables relicTrackables = null;
    VuforiaTrackable relicTemplate = null;

    public void vuforia_init()
    {
        telemetry.addData("Status", "Initializing VuForia.");

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        vuforia_parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vuforia_parameters.vuforiaLicenseKey = Config.VUFORIA_LICENSE_KEY;
        vuforia_parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
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
            telemetry.addData("Status", "Motor %4d %4d %4d",
                    motor.getTargetPosition(), motor.getTargetPosition(),
                    (int) Math.abs(motor.getTargetPosition() - motor.getCurrentPosition()));
            telemetry.update();
        }

        motor.setPower(0);
    }

    private void run_to_position(int fl_tgt, int fr_tgt, int br_tgt, int bl_tgt, double power, int tolerance)
    {
        front_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);

        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        front_left.setTargetPosition(fl_tgt);
        front_right.setTargetPosition(fr_tgt);
        back_left.setTargetPosition(bl_tgt);
        back_right.setTargetPosition(br_tgt);

        front_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        front_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        back_right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        front_left.setPower(power);
        front_right.setPower(power);
        back_right.setPower(power);
        back_left.setPower(power);

        while ( tolerance < Math.abs(front_left.getTargetPosition()  - front_left.getCurrentPosition())  ||
                tolerance < Math.abs(front_right.getTargetPosition() - front_right.getCurrentPosition()) ||
                tolerance < Math.abs(back_right.getTargetPosition()  - back_right.getCurrentPosition())  ||
                tolerance < Math.abs(back_left.getTargetPosition()   - back_left.getCurrentPosition())
                ) {
            telemetry.addData("Status", "FL %4d %4d %4d", front_left.getTargetPosition(), front_left.getTargetPosition(), (int) Math.abs(front_left.getTargetPosition()  - front_left.getCurrentPosition()));
            telemetry.addData("Status", "FR %4d %4d %4d", front_right.getTargetPosition(), front_right.getTargetPosition(), (int) Math.abs(front_right.getTargetPosition()  - front_right.getCurrentPosition()));
            telemetry.addData("Status", "BL %4d %4d %4d", back_left.getTargetPosition(), back_left.getTargetPosition(), (int) Math.abs(back_left.getTargetPosition()  - back_left.getCurrentPosition()));
            telemetry.addData("Status", "BR %4d %4d %4d", back_right.getTargetPosition(), back_right.getTargetPosition(), (int) Math.abs(back_right.getTargetPosition()  - back_right.getCurrentPosition()));
            telemetry.update();
        }

        front_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);
        back_left.setPower(0);
    }
}