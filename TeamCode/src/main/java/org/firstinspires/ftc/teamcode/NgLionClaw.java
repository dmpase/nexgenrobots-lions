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
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * This file contains ...
 */

public class NgLionClaw extends NgClaw {
    /**************************************************************************
     *                           robot configuration                          *
     **************************************************************************/


    // Expansion Hub Portal 1
    //     Expansion Hub 3
    //         Motors
    public static final String LIFT_DRIVE_NAME      = "claw lift";              // Hub 3.Motors[3].Rev Robotics HD Hex

    //         Servos
    public static final String PORT_CLAW_NAME       = "port claw";              // Hub 3.Servos[0].Servo
    public static final String STBD_CLAW_NAME       = "starboard claw";         // Hub 3.Servos[5].Servo



    /**************************************************************************
     *                            robot parameters                            *
     **************************************************************************/


    // Lift parameters
    public static final double LIFT_POWER       = 0.75;
    public static final int    LIFT_TARGET_HI   = 5000;
    public static final int    LIFT_TARGET_INCH =  400;
    public static final int    LIFT_TARGET_LO   =  200;
    public static final int    LIFT_TARGET_SET  =  400;
    public static final DcMotor.Direction LIFT_DIRECTION = DcMotor.Direction.FORWARD;


    // Claw parameters
    public static final double PORT_CLAW_OPENED = 0.90;
    public static final double PORT_CLAW_CLOSED = 0.25;
    public static final double STBD_CLAW_OPENED = 0.00;
    public static final double STBD_CLAW_CLOSED = 0.65;



    // Misc. motor parameters
    public static final int    MOTOR_TARGET_TOLERANCE =   5;    // encoder clicks
    public static final double MOTOR_LAG_SEC          =   0.25; // seconds
    public static final long   MOTOR_LAG_MILLI        = 250;    // milliseconds




    /**************************************************************************
     *                            OpMode functions                            *
     **************************************************************************/



    public NgLionClaw(OpMode op_mode)
    {
        super(op_mode);
    }



    /**************************************************************************
     *                             claw functions                              *
     **************************************************************************/


    // claw and tail servos
    private Servo port_claw = null;
    private Servo stbd_claw = null;

    // claw lift motor
    private DcMotor lift    = null;

    // initialize the claw
    @Override
    public void init()
    {
        if (port_claw == null) {
            port_claw = hardwareMap.get(Servo.class, PORT_CLAW_NAME);
            port_claw.setDirection(Servo.Direction.FORWARD);
        }

        if (stbd_claw == null) {
            stbd_claw = hardwareMap.get(Servo.class, STBD_CLAW_NAME);
            stbd_claw.setDirection(Servo.Direction.FORWARD);
        }

        if (lift == null) {
            lift = hardwareMap.get(DcMotor.class, LIFT_DRIVE_NAME);
            lift.setDirection(LIFT_DIRECTION);
            lift.setPower(0);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    @Override
    public void open()
    {
        port_claw.setPosition(PORT_CLAW_OPENED);
        stbd_claw.setPosition(STBD_CLAW_OPENED);
    }

    @Override
    public void close()
    {
        port_claw.setPosition(PORT_CLAW_CLOSED);
        stbd_claw.setPosition(STBD_CLAW_CLOSED);
    }

    @Override
    public void raise(double height)
    {
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int target = (int) (LIFT_TARGET_LO + height * (LIFT_TARGET_HI - LIFT_TARGET_LO));
        lift.setTargetPosition(target);
        lift.setPower(LIFT_POWER);
    }

    @Override
    public void sync()
    {
        double start = runtime.seconds();
        while ( MOTOR_TARGET_TOLERANCE < Math.abs(lift.getTargetPosition() - lift.getCurrentPosition()) ) {
            if (MOTOR_LAG_SEC < (runtime.seconds() - start)) break;
        }

        if ((runtime.seconds() - start) < MOTOR_LAG_SEC) {
            sleep(MOTOR_LAG_SEC - (start - runtime.seconds()));
        }

        lift.setPower(0);
    }

    @Override
    public void stop()
    {
        lift.setPower(0);
    }
}
