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
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just echoes input from the controllers.
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="HolonomicTestOpMode", group="Iterative Opmode")
// @Disabled
public class HolonomicTestOpMode extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Starting Initialization.");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

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

        // Show the elapsed game time and wheel power.
        telemetry.addData("", "fl = %.2f  fr = %.2f", motors[FRONT_LEFT], motors[FRONT_RIGHT]);
        telemetry.addData("", "bl = %.2f  br = %.2f", motors[BACK_LEFT], motors[BACK_RIGHT]);
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    public static final int MOTOR_COUNT = 4;
    public static final int FRONT_LEFT  = 0;
    public static final int FRONT_RIGHT = 1;
    public static final int BACK_RIGHT  = 2;
    public static final int BACK_LEFT   = 3;

    public double[] compute_motor_settings()
    {
        double stick_dead_zone = 0.1;
        double precision_speed = 0.5;

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
        double alpha = (x == 0 && y == 0) ? 0 : Math.atan2(y, x);
        double bearing = (x == 0 && y == 0) ? 0 : alpha + Math.PI/2.0;
        double throttle = Math.sqrt(x*x + y*y);
        throttle = (1 < throttle) ? 1 : throttle;

        double[] motors = new double[MOTOR_COUNT];

        motors[FRONT_LEFT ] = - throttle * Math.sin(bearing + Math.PI/4);
        motors[FRONT_RIGHT] =   throttle * Math.cos(bearing + Math.PI/4);
        motors[BACK_RIGHT ] =   throttle * Math.sin(bearing + Math.PI/4);
        motors[BACK_LEFT  ] = - throttle * Math.cos(bearing + Math.PI/4);

        return motors;
    }

    public static double radians_to_degrees(double radians)
    {
        return 180*radians/Math.PI;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
