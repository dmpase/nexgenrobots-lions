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

public class NgLionIR extends NgIR {
    /**************************************************************************
     *                           robot configuration                          *
     **************************************************************************/


    // Expansion Hub Portal 1
    //     Expansion Hub 2
    //         Analog Input Devices
    public static final String STBD_BOW_IR_NAME     = "stbd bow ir range";      // Hub 2.Analog Input Devices[0].Analog Input
    public static final String STBD_AFT_IR_NAME     = "stbd aft ir range";      // Hub 2.Analog Input Devices[1].Analog Input
    public static final String PORT_BOW_IR_NAME     = "port bow ir range";      // Hub 2.Analog Input Devices[2].Analog Input
    public static final String PORT_AFT_IR_NAME     = "port aft ir range";      // Hub 2.Analog Input Devices[3].Analog Input



    /**************************************************************************
     *                            OpMode functions                            *
     **************************************************************************/


    public NgLionIR(OpMode op_mode)
    {
        super(op_mode);
    }



    /**************************************************************************
     *                            sensor functions                            *
     **************************************************************************/


    // Pololu IR range sensors
    AnalogInput port_aft_ir = null;
    AnalogInput stbd_aft_ir = null;
    AnalogInput port_bow_ir = null;
    AnalogInput stbd_bow_ir = null;
    Distance    ir_v2in     = new Distance(10.616758844230123, -2.625694922444332, 5.292315651154265);

    @Override
    public void init()
    {
        port_bow_ir = hardwareMap.get(AnalogInput.class, PORT_BOW_IR_NAME);
        stbd_bow_ir = hardwareMap.get(AnalogInput.class, STBD_BOW_IR_NAME);
        stbd_aft_ir = hardwareMap.get(AnalogInput.class, STBD_AFT_IR_NAME);
        port_aft_ir = hardwareMap.get(AnalogInput.class, PORT_AFT_IR_NAME);
    }

    @Override
    public double[] read()
    {
        double[] rv = new double[4];

        rv[PORT_BOW] = ir_v2in.distance(port_bow_ir.getVoltage());
        rv[STBD_BOW] = ir_v2in.distance(stbd_bow_ir.getVoltage());
        rv[STBD_AFT] = ir_v2in.distance(stbd_aft_ir.getVoltage());
        rv[PORT_AFT] = ir_v2in.distance(port_aft_ir.getVoltage());

        return rv;
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {
    }
}
