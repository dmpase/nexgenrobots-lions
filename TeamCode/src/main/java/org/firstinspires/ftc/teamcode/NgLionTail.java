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

public class NgLionTail extends NgTail {
    /**************************************************************************
     *                           robot configuration                          *
     **************************************************************************/


    // Expansion Hub Portal 1
    //     Expansion Hub 2
    //         Servos
    public static final String TAIL_NAME            = "tail";                   // Hub 2.Servos[0].Servo

    //         I2C Bus 3
    public static final String REV_COLOR_RANGE_NAME = "rev color range";        // Hub 2.I2C Bus 3[0].Rev Color/Range Sensor



    /**************************************************************************
     *                            robot parameters                            *
     **************************************************************************/


    // Tail parameters
    public static final double TAIL_POS_UP      = 0.00;
    public static final double TAIL_POS_DN      = 0.90;



    /**************************************************************************
     *                            OpMode functions                            *
     **************************************************************************/


    public NgLionTail(OpMode op_mode)
    {
        super(op_mode);
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {
    }


    /**************************************************************************
     *                              tail functions                            *
     **************************************************************************/


    // tail servos
    private Servo tail = null;

    // initialize the tail
    @Override
    public void init()
    {
        if (tail == null) {
            tail = hardwareMap.get(Servo.class, TAIL_NAME);
            tail.setDirection(Servo.Direction.FORWARD);
        }

        if (color_sensor != null) {
            color_sensor = hardwareMap.get(ColorSensor.class, REV_COLOR_RANGE_NAME);
        }

        if (distance_sensor != null) {
            distance_sensor = hardwareMap.get(DistanceSensor.class, REV_COLOR_RANGE_NAME);
        }
    }

    @Override
    public void raise(double height)
    {
        double pos = TAIL_POS_DN + height * (TAIL_POS_UP - TAIL_POS_DN);
        tail.setPosition(height);
    }


    // REV Robotics distance/color sensor
    private ColorSensor color_sensor = null;
    private DistanceSensor distance_sensor = null;


    @Override
    public double[] color()
    {
        double[] rv = new double[4];

        rv[ALPHA] = color_sensor.alpha();
        rv[RED  ] = color_sensor.red();
        rv[GREEN] = color_sensor.green();
        rv[BLUE ] = color_sensor.blue();

        return rv;
    }

    @Override
    public double distance()
    {
        return distance_sensor.getDistance(DistanceUnit.INCH);
    }
}
