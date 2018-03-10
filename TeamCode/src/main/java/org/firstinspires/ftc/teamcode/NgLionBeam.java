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

public class NgLionBeam extends NgBeam {
    /**************************************************************************
     *                           robot configuration                          *
     **************************************************************************/


    // Expansion Hub Portal 1
    //     Expansion Hub 2
    //         Servos
    public static final String BEAM_SWIVEL_NAME     = "beam swivel";            // Hub 2.Servos[3].Servo
    public static final String BEAM_CLAW_NAME       = "beam claw";              // Hub 2.Servos[5].Servo


    //     Expansion Hub 3
    //         Motors
    public static final String BEAM_DRIVE_NAME      = "beam drive";             // Hub 3.Motors[2].Rev Robotics HD Hex



    /**************************************************************************
     *                            robot parameters                            *
     **************************************************************************/


    // Beam parameters
    public static final double BEAM_POWER       = 0.40;
    public static final int    BEAM_TARGET_IN   =    0;         // encoder clicks for full retraction
    public static final int    BEAM_TARGET_OUT  = -15000;       // encoder clicks for full extension
    public static final DcMotor.Direction BEAM_DIRECTION = DcMotor.Direction.FORWARD;
    public static final double BEAM_CLAW_OPENED = 0.00;
    public static final double BEAM_CLAW_CLOSED = 0.90;
    public static final double BEAM_SWIVEL_UP   = 0.50;
    public static final double BEAM_SWIVEL_DOWN = 0.00;



    /**************************************************************************
     *                            OpMode functions                            *
     **************************************************************************/


    public NgLionBeam(OpMode op_mode)
    {
        super(op_mode);
    }


    private ElapsedTime runtime = new ElapsedTime();


    /**************************************************************************
     *                              beam functions                            *
     **************************************************************************/


    // beam motor
    private DcMotor beam_drive  = null;

    // beam servos
    private Servo beam_claw     = null;
    private Servo beam_swivel   = null;

    // initialize the beam
    @Override
    public void init()
    {
        if (beam_claw == null) {
            beam_claw = hardwareMap.get(Servo.class, BEAM_CLAW_NAME);
            beam_claw.setDirection(Servo.Direction.FORWARD);
        }

        if (beam_swivel == null) {
            beam_swivel = hardwareMap.get(Servo.class, BEAM_SWIVEL_NAME);
            beam_swivel.setDirection(Servo.Direction.FORWARD);
        }

        if (beam_drive == null) {
            beam_drive = hardwareMap.get(DcMotor.class, BEAM_DRIVE_NAME);
            beam_drive.setDirection(DcMotor.Direction.FORWARD);
            beam_drive.setPower(0);
            beam_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            beam_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    @Override
    public void open()
    {
        beam_claw.setPosition(BEAM_CLAW_OPENED);
    }

    @Override
    public void close()
    {
        beam_claw.setPosition(BEAM_CLAW_CLOSED);
    }

    @Override
    public void raise()
    {
        beam_swivel.setPosition(BEAM_SWIVEL_UP);
    }

    @Override
    public void lower()
    {
        beam_swivel.setPosition(BEAM_SWIVEL_DOWN);
    }

    @Override
    public void extend(double length)
    {
        beam_drive.setPower(0);
        int target = (int) (BEAM_TARGET_IN + length * (BEAM_TARGET_OUT - BEAM_TARGET_IN));
        beam_drive.setTargetPosition(target);
        beam_drive.setPower(BEAM_POWER);
    }

    @Override
    public void stop()
    {
        beam_drive.setPower(0);
    }


    // Misc. motor parameters
    public static final int    MOTOR_TARGET_TOLERANCE =   5;    // encoder clicks
    public static final double MOTOR_LAG_SEC          =   0.25; // seconds

    @Override
    public void sync()
    {
        double start = runtime.seconds();
        while ( MOTOR_TARGET_TOLERANCE < Math.abs(beam_drive.getTargetPosition() - beam_drive.getCurrentPosition()) ) {
            if (MOTOR_LAG_SEC < (runtime.seconds() - start)) break;
        }

        if ((runtime.seconds() - start) < MOTOR_LAG_SEC) {
            sleep(MOTOR_LAG_SEC - (start - runtime.seconds()));
        }
    }
}
