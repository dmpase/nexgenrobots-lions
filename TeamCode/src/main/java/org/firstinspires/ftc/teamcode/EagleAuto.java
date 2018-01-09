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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Random;

import static org.firstinspires.ftc.teamcode.Config.BALANCING_STONE;
import static org.firstinspires.ftc.teamcode.Config.PLAYING_FIELD;
import static org.firstinspires.ftc.teamcode.Config.ROTATION_RATE;

/**
 * This file contains the Griffin Lions and Griffin Eagles team Autonomous Op Mode.
 *
 * See the Griffin Auto Input User Guide for details on Op Mode controls.
 */

@Autonomous(name="Eagle Auto", group="Autonomous")
// @Disabled
public class EagleAuto extends LinearOpMode {
    // Declare OpMode members.

    public static enum Color {BLUE, RED, UNKNOWN}

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        telemetry.addData("Status", "Starting Initialization.");

        drive_init();           // initialize the drive motors

        servo_init();           // initialize the claw servos (and lift motor)

        sensor_init();          // initialize the sensors (rev color/distance)

        vuforia_init();         // initialize the vuforia subsystem

        // Send telemetry message to indicate successful encoder, servo and sensor reset
        telemetry.addData("Status", "Initialization Complete.");
        telemetry.update();

        // make minor adjustments to position, as needed, and prompt the user for input
        telemetry.addData("Move", "Stick/bumper/trigger to move.");
        telemetry.addData("Claw", "X/B/Y/A to open/close/raise/lower.");
        telemetry.addData("Tail", "DPAD up/down to raise/lower.");
        telemetry.addData("Start", "Press 'Guide' to select quadrant.");
        telemetry.update();
        Object[] open_claw  = {Command.OPEN_CLAW,                           };
        Object[] close_claw = {Command.CLOSE_CLAW,                          };
        Object[] raise_claw = {Command.LIFT,        Config.LIFT_TARGET_INCH };
        Object[] lower_claw = {Command.LIFT,        Config.LIFT_TARGET_LO   };
        Object[] reset_claw = {Command.LIFT,        Config.LIFT_TARGET_SET  };

        while (! gamepad1.guide) {
            get_motor_settings();

            if (gamepad1.x || gamepad2.x) {
                execute(open_claw);
            } else if (gamepad1.b || gamepad2.b) {
                execute(close_claw);
            } else if (gamepad1.y || gamepad2.y) {
                execute(raise_claw);
            } else if (gamepad1.a || gamepad2.a) {
                execute(lower_claw);
            } else if (gamepad1.dpad_up || gamepad2.dpad_up) {
                tail.setPosition(Config.TAIL_POS_UP);
            } else if (gamepad1.dpad_down || gamepad2.dpad_down) {
                tail.setPosition(Config.TAIL_POS_DN);
            }
        }

        // make sure the tail is in the up position
        tail.setPosition(Config.TAIL_POS_UP);
        execute(raise_claw);


        // select the team color and playing field quadrant
        Color team_color = Color.UNKNOWN;
        int quadrant = UNKNOWN;

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
            telemetry.addData("Blue", "'X' for left, 'A' for right.");
            telemetry.addData("Red", "'Y' for left, 'B' for right.");
            telemetry.addData("Selection", "Quadrant:%s, Team:%s",
                    QUADRANT_NAME[quadrant], team_color.toString());
            telemetry.addData("Start", "Press 'Start' to wait for start.");
            telemetry.update();
        }


        // team color and quadrant have been selected, waiting for start of the game
        telemetry.addData("Selection", "Quadrant:%s, Team:%s",
                QUADRANT_NAME[quadrant], team_color.toString());
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
            vuforia_result = VUFORIA_LEFT;
        } else if (vumark_position.equalsIgnoreCase("CENTER")) {
            vuforia_result = VUFORIA_CENTER;
        } else if (vumark_position.equalsIgnoreCase("RIGHT")) {
            vuforia_result = VUFORIA_RIGHT;
        }

        telemetry.addData("VuMark", "%s", vuMark.name());
        telemetry.update();


        // ^^^ dislodge the jewell (or not...)
        // lower tail and pause to get a good read on the jewell color
        tail.setPosition(Config.TAIL_POS_DN);
        sleep(1000);

        // read the jewell color
        Color jewell_color = Color.UNKNOWN;
        if (color_sensor.red() < color_sensor.blue()) {
            jewell_color = Color.BLUE;
        } else if (color_sensor.blue() < color_sensor.red()) {
            jewell_color = Color.RED;
        }

        telemetry.addData("Jewell", "Team: %s, Jewell:%s", team_color.name(), jewell_color.name());
        telemetry.update();

        // dislodge the opposing alliance jewell from its stand
        // if jewell color == team color, clockwise then counter clockwise, else opposite
        Object[] clockwise        = {Command.ROTATE, -30.0, AUTO_PWR, AUTO_TOL, ROTATION_RATE};
        Object[] counterclockwise = {Command.ROTATE, +30.0, AUTO_PWR, AUTO_TOL, ROTATION_RATE};
        if (team_color == Color.UNKNOWN || jewell_color == Color.UNKNOWN) {
            ;
        } else if (team_color == jewell_color) {
            execute(counterclockwise);
            tail.setPosition(Config.TAIL_POS_UP);
            execute(clockwise);
        } else {
            execute(clockwise);
            tail.setPosition(Config.TAIL_POS_UP);
            execute(counterclockwise);
        }
        sleep(500);


        // ^^^ deliver the block to the crypto box
        execute(quad_cmds[quadrant]);


        // start dancing!
        Random rand = new Random();
        int idx = (int) (rand.nextDouble()*dance_cmds.length);
        if (dance_cmds != null && 0 <= idx && idx < dance_cmds.length && dance_cmds[idx] != null) {
            telemetry.addData("", "Let's dance!");
            telemetry.update();
            execute(dance_cmds[idx]);
        }
        // dance is over.

        telemetry.addData("Path", "Complete");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.update();
    }


    private static final double AUTO_PWR = 0.08;
    private static final double FAST_PWR = 0.25;
    private static final int    AUTO_TOL = 10;

    // blue-left quadrant command sequence
    private static final Object[][] blue_left_cmd = {
            {Command.ROTATE,    90.0, AUTO_PWR, AUTO_TOL, ROTATION_RATE     },  // turn towards crypto box
            {Command.FORWARD,   18.0, AUTO_PWR, AUTO_TOL, BALANCING_STONE   },  // move off the stone - 18.0" (22.0)
            {Command.FORWARD,    2.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // calibrate position
            {Command.BACKWARD,   4.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },
            {Command.FORWARD,    6.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // align with top of triangle
            {Command.STBD,      15.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // move to crypto box center
            {Command.ADJUST,     6.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // adjust for VuForia VuMark
            {Command.FORWARD,    5.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // place glyph in crypto box - 5.0" (8.0)
            {Command.OPEN_CLAW,                                             },  // release glyph
            {Command.BACKWARD,   5.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // move away from crypto box - 5.0"
            {Command.CLOSE_CLAW,                                            },
            {Command.LIFT,        Config.LIFT_TARGET_LO                     },  // lower the claw
    };

    // blue-right quadrant command sequence
    private static final Object[][] blue_right_cmd = {
            {Command.ROTATE,    90.0, AUTO_PWR, AUTO_TOL, ROTATION_RATE     },  // turn towards crypto box
            {Command.FORWARD,   18.0, AUTO_PWR, AUTO_TOL, BALANCING_STONE   },  // move off the stone - 18.0" (22.0)
            {Command.FORWARD,    4.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // calibrate position
            {Command.BACKWARD,   4.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },
            {Command.FORWARD,   15.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // move to crypto box center - 15.0" (22.0)
            {Command.ROTATE,    90.0, AUTO_PWR, AUTO_TOL, ROTATION_RATE     },  // turn to face crypto box
            {Command.ADJUST,     6.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // adjust for VuForia VuMark
            {Command.FORWARD,    5.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // place glyph in crypto box - 5.0" (8.0)
            {Command.OPEN_CLAW,                                             },  // release glyph
            {Command.BACKWARD,   5.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // move away from crypto box - 5.0"
            {Command.CLOSE_CLAW,                                            },
            {Command.LIFT,        Config.LIFT_TARGET_LO                     },  // lower the claw
    };

    // red-left quadrant command sequence
    private static final Object[][] red_left_cmd = {
            {Command.ROTATE,   -90.0, AUTO_PWR, AUTO_TOL, ROTATION_RATE     },  // turn towards crypto box
            {Command.FORWARD,   15.0, AUTO_PWR, AUTO_TOL, BALANCING_STONE   },  // move off the stone - 18.0" (22.0)
            {Command.FORWARD,    4.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // calibrate position
            {Command.BACKWARD,   4.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },
            {Command.FORWARD,   24.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // move to crypto box center - 15.0 (22.0)
            {Command.ROTATE,   -90.0, AUTO_PWR, AUTO_TOL, ROTATION_RATE     },  // turn to face crypto box
            {Command.ADJUST,     6.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // adjust for VuForia VuMark
            {Command.FORWARD,    5.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // place glyph in crypto box - 5.0 (8.0)
            {Command.OPEN_CLAW,                                             },  // release glyph
            {Command.BACKWARD,   5.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // move away from crypto box - 5.0
            {Command.CLOSE_CLAW,                                            },
            {Command.LIFT,        Config.LIFT_TARGET_LO                     },  // lower the claw
    };

    // red-right quadrant command sequence
    private static final Object[][] red_right_cmd = {
            {Command.ROTATE,   -90.0, AUTO_PWR, AUTO_TOL, ROTATION_RATE     },  // turn towards crypto box
            {Command.FORWARD,   18.0, AUTO_PWR, AUTO_TOL, BALANCING_STONE   },  // move off the stone - 18.0" (22.0)
            {Command.FORWARD,    4.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // calibrate position
            {Command.BACKWARD,   4.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },
            {Command.FORWARD,    6.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // align with top of triangle
            {Command.PORT,      12.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // move to crypto box center
            {Command.ADJUST,     6.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // adjust for VuForia VuMark
            {Command.FORWARD,    5.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // place glyph in crypto box - 5.0" (8.0)
            {Command.OPEN_CLAW,                                             },  // release glyph
            {Command.BACKWARD,   5.0, AUTO_PWR, AUTO_TOL, PLAYING_FIELD     },  // move away from crypto box - 5.0"
            {Command.CLOSE_CLAW,                                            },
            {Command.LIFT,        Config.LIFT_TARGET_LO                     },  // lower the claw
    };


    private static final Object[][] dance_0_cmd = {
            {Command.SLEEP,      1.0,                                       },
            {Command.OPEN_CLAW,                                             },
            {Command.CLOSE_CLAW,  Command.PORT,                             },
            {Command.OPEN_CLAW,                                             },
            {Command.ROTATE,   -10.0, AUTO_PWR, AUTO_TOL, ROTATION_RATE     },
            {Command.SLEEP,      0.2,                                       },
            {Command.ROTATE,   370.0, FAST_PWR, AUTO_TOL, ROTATION_RATE     },
            {Command.CLOSE_CLAW,  Command.STBD,                             },
            {Command.SLEEP,      1.0,                                       },
            {Command.OPEN_CLAW,                                             },
            {Command.CLOSE_CLAW,                                            },
    };

    private static final Object[][] dance_1_cmd = {
            {Command.SLEEP,      0.5,                                       },
            {Command.ROTATE,   -15.0, AUTO_PWR, AUTO_TOL, ROTATION_RATE     },
            {Command.SLEEP,      0.5,                                       },
            {Command.ROTATE,   -15.0, AUTO_PWR, AUTO_TOL, ROTATION_RATE     },
            {Command.SLEEP,      0.5,                                       },
            {Command.ROTATE,   -15.0, AUTO_PWR, AUTO_TOL, ROTATION_RATE     },
            {Command.SLEEP,      0.5,                                       },
            {Command.ROTATE,   405.0, FAST_PWR, AUTO_TOL, ROTATION_RATE     },
            {Command.OPEN_CLAW,                                             },
            {Command.CLOSE_CLAW,                                            },
    };

    private static final Object[][][] dance_cmds = {
            null,
//            dance_0_cmd,
            null,
//            dance_1_cmd,
            null,
    };

    private static final int UNKNOWN    = 0;
    private static final int BLUE_LEFT  = 1;
    private static final int BLUE_RIGHT = 2;
    private static final int RED_LEFT   = 3;
    private static final int RED_RIGHT  = 4;
    private static final String[] QUADRANT_NAME = {"UNKNOWN", "BLUE LEFT", "BLUE RIGHT", "RED LEFT", "RED RIGHT"};

    private static final Object[][][] quad_cmds = {
            null,
            blue_left_cmd,
            blue_right_cmd,
            red_left_cmd,
            red_right_cmd,
    };


    // command operation codes (i.e., op codes)
    public static enum Command {ROTATE, FORWARD, BACKWARD, PORT, STBD, ADJUST, OPEN_CLAW, CLOSE_CLAW, LIFT, SLEEP}

    // array indexes into a command array
    private static final int OPCODE     = 0;
    private static final int ANGLE      = 1;
    private static final int INCHES     = 1;
    private static final int TARGET     = 1;
    private static final int SECONDS    = 1;
    private static final int POWER      = 2;
    private static final int TOLERANCE  = 3;
    private static final int SURFACE    = 4;

    // position indicators for LEFT, CENTER and RIGHT vumark images
    private static final int VUFORIA_LEFT   = -1;
    private static final int VUFORIA_CENTER = 0;
    private static final int VUFORIA_RIGHT  = 1;
    private int vuforia_result = VUFORIA_CENTER;

    // execute a sequence of robot commands
    private void execute(Object[][] cmd)
    {
        for (int i=0; cmd != null && i < cmd.length; i++) {
            execute(cmd[i]);
        }
    }

    // execute a single robot command
    private void execute(Object[] cmd)
    {
        // if cmd == null, do nothing because there is no command to execute
        if (cmd == null) return;

        Command op_code = (Command) cmd[OPCODE];
        if (op_code == Command.ROTATE) {
            // rotate the robot, angle is in degrees, positive angle is counterclockwise
            double angle                = (double) cmd[ANGLE];
            double power                = (double) cmd[POWER];
            int    tolerance            = (int)    cmd[TOLERANCE];
            double degrees_to_clicks    = (double) cmd[SURFACE];
            int    clicks               = (int)    (degrees_to_clicks * angle);
            run_to_position(clicks, clicks, clicks, clicks, power, tolerance);
        } else if (op_code == Command.FORWARD) {
            // move the robot forward, distance is in inches
            double distance             = (double) cmd[INCHES];
            double power                = (double) cmd[POWER];
            int    tolerance            = (int)    cmd[TOLERANCE];
            double distance_to_clicks   = (double) cmd[SURFACE];
            int    clicks               = (int)    (distance_to_clicks * distance);
            run_to_position(-clicks, clicks, clicks, -clicks, power, tolerance);
        } else if (op_code == Command.BACKWARD) {
            // move the robot backward, distance is in inches
            double distance             = (double) cmd[INCHES];
            double power                = (double) cmd[POWER];
            int    tolerance            = (int)    cmd[TOLERANCE];
            double distance_to_clicks   = (double) cmd[SURFACE];
            int    clicks               = (int)    (distance_to_clicks * distance);
            run_to_position(clicks, -clicks, -clicks, clicks, power, tolerance);
        } else if (op_code == Command.PORT) {
            // move the robot laterally to port, distance is in inches
            double distance             = (double) cmd[INCHES];
            double power                = (double) cmd[POWER];
            int    tolerance            = (int)    cmd[TOLERANCE];
            double distance_to_clicks   = (double) cmd[SURFACE];
            int    clicks               = (int)    (distance_to_clicks * distance);
            run_to_position(clicks, clicks, -clicks, -clicks, power, tolerance);
        } else if (op_code == Command.STBD) {
            // move the robot laterally to starboard, distance is in inches
            double distance             = (double) cmd[INCHES];
            double power                = (double) cmd[POWER];
            int    tolerance            = (int)    cmd[TOLERANCE];
            double distance_to_clicks   = (double) cmd[SURFACE];
            int    clicks               = (int)    (distance_to_clicks * distance);
            run_to_position(-clicks, -clicks, clicks, clicks, power, tolerance);
        } else if (op_code == Command.ADJUST) {
            // adjust the robot position (laterally, to port or starboard) based on the VuForia VuMark
            // distance is in inches, viewforia_result contains -1 (left), 0 (center), or 1 (right)
            double distance             = vuforia_result * (double) cmd[INCHES];
            double power                = (double) cmd[POWER];
            int    tolerance            = (int)    cmd[TOLERANCE];
            double distance_to_clicks   = (double) cmd[SURFACE];
            int    clicks               = (int)    (distance_to_clicks * distance);
            run_to_position(-clicks, -clicks, clicks, clicks, power, tolerance);
        } else if (op_code == Command.OPEN_CLAW) {
            // open the claw, both sides if there is no qualifier, or just port or starboard if qualifier is included
            if (cmd.length == 1) {
                port_claw.setPosition(Config.PORT_CLAW_OPENED);
                stbd_claw.setPosition(Config.STBD_CLAW_OPENED);
            } else if (cmd.length == 2 && (Command) cmd[TARGET] == Command.PORT) {
                port_claw.setPosition(Config.PORT_CLAW_OPENED);
            } else if (cmd.length == 2 && (Command) cmd[TARGET] == Command.STBD) {
                stbd_claw.setPosition(Config.STBD_CLAW_OPENED);
            }
            sleep(Config.MOTOR_LAG_MILLI);
        } else if (op_code == Command.CLOSE_CLAW) {
            // close the claw, both sides if there is no qualifier, or just port or starboard if qualifier is included
            if (cmd.length == 1) {
                port_claw.setPosition(Config.PORT_CLAW_CLOSED);
                stbd_claw.setPosition(Config.STBD_CLAW_CLOSED);
            } else if (cmd.length == 2 && (Command) cmd[TARGET] == Command.PORT) {
                port_claw.setPosition(Config.PORT_CLAW_CLOSED);
            } else if (cmd.length == 2 && (Command) cmd[TARGET] == Command.STBD) {
                stbd_claw.setPosition(Config.STBD_CLAW_CLOSED);
            }
            sleep(Config.MOTOR_LAG_MILLI);
        } else if (op_code == Command.LIFT) {
            // raise or lower the claw lift
            int target = (int) cmd[TARGET];
            run_to_position(lift, target, Config.LIFT_POWER, Config.MOTOR_TARGET_TOLERANCE);
        } else if (op_code == Command.SLEEP) {
            // take a rest to wait for motors and servos to catch up
            double seconds = (double) cmd[SECONDS];
            sleep(seconds);
        }
    }

    // robot drive motors. mapping follows, relative to a person at the center facing forward
    // port bow      == front left
    // starboard bow == front right
    // starboard aft == back right
    // port aft      == back left
    private DcMotor port_bow_drive = null;
    private DcMotor stbd_bow_drive = null;
    private DcMotor stbd_aft_drive = null;
    private DcMotor port_aft_drive = null;

    // initialize the drive motors
    public void drive_init()
    {
        telemetry.addData("Status", "Initializing Motors.");

        // drive motors are of type DcMotor
        port_bow_drive = hardwareMap.get(DcMotor.class, Config.PORT_BOW);
        stbd_bow_drive = hardwareMap.get(DcMotor.class, Config.STBD_BOW);
        stbd_aft_drive = hardwareMap.get(DcMotor.class, Config.STBD_AFT);
        port_aft_drive = hardwareMap.get(DcMotor.class, Config.PORT_AFT);

        // make sure all motors are stopped
        port_bow_drive.setPower(0);
        stbd_bow_drive.setPower(0);
        stbd_aft_drive.setPower(0);
        port_aft_drive.setPower(0);

        // drive direction is FORWARD, adjustments for direction are made when used
        port_bow_drive.setDirection(DcMotor.Direction.FORWARD);
        stbd_bow_drive.setDirection(DcMotor.Direction.FORWARD);
        stbd_aft_drive.setDirection(DcMotor.Direction.FORWARD);
        port_aft_drive.setDirection(DcMotor.Direction.FORWARD);

        // reset the encoders to zero
        port_bow_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stbd_bow_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stbd_aft_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        port_aft_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    // claw and tail servos
    private Servo port_claw = null;
    private Servo stbd_claw = null;
    private Servo tail      = null;

    // claw lift motor
    private DcMotor lift    = null;

    // initialize the tail and claw servos plus the claw lift motor
    // do not initialize the beam motor or servos because the beam is not used in autonomous mode
    public void servo_init()
    {
        port_claw = hardwareMap.get(Servo.class, Config.PORT_CLAW);
        port_claw.setDirection(Servo.Direction.FORWARD);

        stbd_claw = hardwareMap.get(Servo.class, Config.STBD_CLAW);
        stbd_claw.setDirection(Servo.Direction.FORWARD);

        tail  = hardwareMap.get(Servo.class, Config.TAIL);
        tail.setDirection(Servo.Direction.FORWARD);

        lift = hardwareMap.get(DcMotor.class, Config.LIFT_DRIVE);
        lift.setDirection(Config.LIFT_DIRECTION);
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    // REV Robotics distance/color sensor
    private ColorSensor color_sensor = null;
    private DistanceSensor distance_sensor = null;

    // initialize the tail color sensor
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

    // initialize the vuforia mechanism
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


    // run a single motor (e.g., claw lift) to a given position,
    // do not reset the encoder before starting or when done
    private void run_to_position(DcMotor motor, int tgt, double power, int tolerance)
    {
        // set the target position
        motor.setPower(0);
        motor.setTargetPosition(tgt);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // tell the motor to begin moving
        motor.setPower(power);

        // wait for the motor to reach its target
        while ( tolerance < Math.abs(motor.getTargetPosition()  - motor.getCurrentPosition()) ) {
            ;
        }

        // wait for the motor to settle in to its position
        sleep(Config.MOTOR_LAG_MILLI);

        // cut the power
        motor.setPower(0);
    }

    // run the drive motors to a given position,
    // DO reset the encoder before starting (i.e., always start from zero)
    private void run_to_position(int fl_tgt, int fr_tgt, int br_tgt, int bl_tgt, double power, int tolerance)
    {
        // cut all power to the motors
        port_bow_drive.setPower(0);
        stbd_bow_drive.setPower(0);
        stbd_aft_drive.setPower(0);
        port_aft_drive.setPower(0);

        // reset the encoders to zero
        port_bow_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stbd_bow_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        port_aft_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        stbd_aft_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set the new target position
        port_bow_drive.setTargetPosition(fl_tgt);
        stbd_bow_drive.setTargetPosition(fr_tgt);
        port_aft_drive.setTargetPosition(bl_tgt);
        stbd_aft_drive.setTargetPosition(br_tgt);

        // tell the motors we want them to run to the target position
        port_bow_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stbd_bow_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        port_aft_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        stbd_aft_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // apply power to the motors
        port_bow_drive.setPower(power);
        stbd_bow_drive.setPower(power);
        stbd_aft_drive.setPower(power);
        port_aft_drive.setPower(power);

        // wait for all motors to reach their position
        while ( tolerance < Math.abs(port_bow_drive.getTargetPosition() - port_bow_drive.getCurrentPosition()) ||
                tolerance < Math.abs(stbd_bow_drive.getTargetPosition() - stbd_bow_drive.getCurrentPosition()) ||
                tolerance < Math.abs(stbd_aft_drive.getTargetPosition() - stbd_aft_drive.getCurrentPosition()) ||
                tolerance < Math.abs(port_aft_drive.getTargetPosition() - port_aft_drive.getCurrentPosition())) {
            ;
        }

        // sleep while the motors settle into their new position
        sleep(Config.MOTOR_LAG_MILLI);

        // kill all power to the motors
        port_bow_drive.setPower(0);
        stbd_bow_drive.setPower(0);
        stbd_aft_drive.setPower(0);
        port_aft_drive.setPower(0);
    }


    public static final double POWER_LIMIT = 0.95;

    public static final double stick_dead_zone   = 0.05;
    public static final double full_speed        = 0.10;
    public static final double half_speed        = 0.05;

    public static final double trigger_dead_zone = 0.05;
    public static final double slow_turn         = 0.05;

    public static final int PORT_BOW             = 0;
    public static final int STBD_BOW             = 1;
    public static final int STBD_AFT             = 2;
    public static final int PORT_AFT             = 3;
    public static final int MOTOR_COUNT          = 4;

    // read input from the game pad and use it to adjust the position of the robot
    // this is used for fine adjustments to placement on the balancing stone before starting the game
    public void get_motor_settings()
    {
        // use game pad 1 right stick to determinie speed and bearing UNLESS the left stick is being used
        // right stick is for speed, left stick is for precision
        double x = (-stick_dead_zone < gamepad1.left_stick_x && gamepad1.left_stick_x < stick_dead_zone) ? 0 : full_speed * gamepad1.left_stick_x;
        double y = (-stick_dead_zone < gamepad1.left_stick_y && gamepad1.left_stick_y < stick_dead_zone) ? 0 : full_speed * gamepad1.left_stick_y;
        if (gamepad1.right_stick_x < -stick_dead_zone || stick_dead_zone < gamepad1.right_stick_x ||
                gamepad1.right_stick_y < -stick_dead_zone || stick_dead_zone < gamepad1.right_stick_y) {

            x = half_speed * gamepad1.right_stick_x;
            y = half_speed * gamepad1.right_stick_y;
        }

        // get the angle of the stick to compute the desired bearing
        double alpha   = (x == 0 && y == 0) ? 0 : Math.atan2(y, x);
        double bearing = (x == 0 && y == 0) ? 0 : alpha + Math.PI/2.0;

        // limit the throttle
        double throttle = Math.sqrt(x*x + y*y);
        throttle = (POWER_LIMIT < throttle) ? POWER_LIMIT : throttle;

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
            rotation = slow_turn * gamepad1.left_trigger;
        } else if (trigger_dead_zone < gamepad1.right_trigger) {
            // turn right using trigger
            rotation = -slow_turn * gamepad1.right_trigger;
        }

        double[] motors = new double[MOTOR_COUNT];

        // use the holonomic drive equations to determine power to the motors
        motors[PORT_BOW] = - throttle * Math.sin(bearing + Math.PI/4) + rotation;
        motors[STBD_BOW] =   throttle * Math.cos(bearing + Math.PI/4) + rotation;
        motors[STBD_AFT] =   throttle * Math.sin(bearing + Math.PI/4) + rotation;
        motors[PORT_AFT] = - throttle * Math.cos(bearing + Math.PI/4) + rotation;

        // limit the motors to -1.0 <= motor <= 1.0
        // scale them evenly if adjustments are made
        if (motors[PORT_BOW] < -POWER_LIMIT || POWER_LIMIT < motors[PORT_BOW] ||
                motors[STBD_BOW] < -POWER_LIMIT || POWER_LIMIT < motors[STBD_BOW] ||
                motors[PORT_AFT] < -POWER_LIMIT || POWER_LIMIT < motors[PORT_AFT] ||
                motors[STBD_AFT] < -POWER_LIMIT || POWER_LIMIT < motors[STBD_AFT]) {

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

        // explicitly set the motor run mode to analog
        port_bow_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stbd_bow_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        stbd_aft_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        port_aft_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // apply power to the drive motors
        port_bow_drive.setPower(motors[PORT_BOW]);
        stbd_bow_drive.setPower(motors[STBD_BOW]);
        stbd_aft_drive.setPower(motors[STBD_AFT]);
        port_aft_drive.setPower(motors[PORT_AFT]);
    }

    // take a quick snooze, catch 40 winks, etc...
    public static void sleep(double sec)
    {
        long ms = (long)(sec * 1000);
        try {
            Thread.sleep(ms);
        } catch (Exception e) {
            ;
        }
    }
}