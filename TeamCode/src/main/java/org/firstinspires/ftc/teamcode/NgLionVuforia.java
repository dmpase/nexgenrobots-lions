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

public class NgLionVuforia extends NgVuforia {
    /**************************************************************************
     *                            robot parameters                            *
     **************************************************************************/


    // VuForia License Key
    public static final String VUFORIA_LICENSE_KEY = "AWVXYZn/////AAAAGcG6g8XSSUMJsDaizcApOtsaA0fWzUQwImrdEn1MqH4JNqCzUwlyvEX0YALy7XyUeSpiANJkBg9kplUtcniUZKw8bF0dSpEfXZKXxn1yhbIohmpVmIK+Ngv1imYrkY6ePmvTfO2IpyQi5yO5ZmfSC8OzlH+XEMD0vRIXHMhxFpin7vTIHaoz8MEifSjRTznh1ZUSRnJfQ01KvMHEefES0kwhehlEKoqgpNMOYg0B5pV0bDDi9/Qh4eMR7sEk1GSx3QPxl/lYuZVcWSh8DutXv8oo9LhnbAaHTecCAR6gnNODow0WUAH2N9vxdLOjk2UfWVEJgqmHembIDHRzJN4fjcOECTFfLHIVmZ66GwgjPWxV";
    public static final VuforiaLocalizer.CameraDirection CAMERA_DIRECTION = VuforiaLocalizer.CameraDirection.FRONT;



    /**************************************************************************
     *                            OpMode functions                            *
     **************************************************************************/


    public NgLionVuforia(OpMode op_mode)
    {
        super(op_mode);
    }


    /**************************************************************************
     *                           vuforia functions                            *
     **************************************************************************/


    // VuForia objects
    int cameraMonitorViewId = -1;
    VuforiaLocalizer vuforia = null;
    VuforiaLocalizer.Parameters vuforia_parameters = null;
    VuforiaTrackables relicTrackables = null;
    VuforiaTrackable relicTemplate = null;
    RelicRecoveryVuMark vuMark = null;

    // initialize the vuforia mechanism
    @Override
    public void init()
    {
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        vuforia_parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vuforia_parameters.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforia_parameters.cameraDirection = CAMERA_DIRECTION;
        vuforia = ClassFactory.createVuforiaLocalizer(vuforia_parameters);
        relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate");

        relicTrackables.activate();
    }

    @Override
    public int read()
    {
        int vuforia_result = CENTER;

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        String vumark_position = vuMark.name();
        if (vumark_position.equalsIgnoreCase("LEFT")) {
            vuforia_result = LEFT;
        } else if (vumark_position.equalsIgnoreCase("CENTER")) {
            vuforia_result = CENTER;
        } else if (vumark_position.equalsIgnoreCase("RIGHT")) {
            vuforia_result = RIGHT;
        }

        return vuforia_result;
    }


    @Override
    public void stop()
    {
    }
}
