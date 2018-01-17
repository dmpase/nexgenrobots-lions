package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by Doug on 12/24/2017.
 */

public class EagleConfig {
    // Expansion Hub Portal 1
    //     Expansion Hub 2
    //         Motors
    public static final String PORT_BOW         = "front left";             // Hub 2.Motors[0].Rev Robotics HD Hex
    public static final String STBD_BOW         = "front right";            // Hub 2.Motors[1].Rev Robotics HD Hex
    public static final String STBD_AFT         = "back right";             // Hub 2.Motors[2].Rev Robotics HD Hex
    public static final String PORT_AFT         = "back left";              // Hub 2.Motors[3].Rev Robotics HD Hex

    //         Servos
    public static final String CLAW             = "claw";                   // Hub 2.Servos[0].Servo
    public static final String TAIL             = "tail";                   // Hub 2.Servos[1].Servo
    public static final String BEAM_SWIVEL      = "beam swivel";            // Hub 2.Servos[3].Servo
    public static final String BEAM_CLAW        = "beam claw";              // Hub 2.Servos[5].Servo

    //         I2C Bus 3
    public static final String REV_COLOR_RANGE  = "rev color range";        // Hub 2.I2C Bus 3[0].Rev Color/Range Sensor


    //     Expansion Hub 3
    //         Motors
    public static final String BEAM_DRIVE       = "beam drive";             // Hub 3.Motors[2].Rev Robotics HD Hex
    public static final String LIFT_DRIVE       = "claw lift";              // Hub 3.Motors[3].Rev Robotics HD Hex


    // VuForia License Key
    public static final String VUFORIA_LICENSE_KEY = "AWVXYZn/////AAAAGcG6g8XSSUMJsDaizcApOtsaA0fWzUQwImrdEn1MqH4JNqCzUwlyvEX0YALy7XyUeSpiANJkBg9kplUtcniUZKw8bF0dSpEfXZKXxn1yhbIohmpVmIK+Ngv1imYrkY6ePmvTfO2IpyQi5yO5ZmfSC8OzlH+XEMD0vRIXHMhxFpin7vTIHaoz8MEifSjRTznh1ZUSRnJfQ01KvMHEefES0kwhehlEKoqgpNMOYg0B5pV0bDDi9/Qh4eMR7sEk1GSx3QPxl/lYuZVcWSh8DutXv8oo9LhnbAaHTecCAR6gnNODow0WUAH2N9vxdLOjk2UfWVEJgqmHembIDHRzJN4fjcOECTFfLHIVmZ66GwgjPWxV";
    public static final VuforiaLocalizer.CameraDirection CAMERA_DIRECTION = VuforiaLocalizer.CameraDirection.FRONT;


    // Lift parameters
    public static final double LIFT_POWER       = 0.75;
    public static final int    LIFT_TARGET_HI   = 4800;
    public static final int    LIFT_TARGET_INCH =  400;
    public static final int    LIFT_TARGET_LO   =    0;
    public static final int    LIFT_TARGET_SET  =  400;
    public static final DcMotor.Direction LIFT_DIRECTION = DcMotor.Direction.FORWARD;


    // Claw parameters
    public static final double CLAW_OPENED      =  1.00;
    public static final double CLAW_CLOSED      = -0.50;


    // Tail parameters
    public static final double TAIL_POS_UP      = 0.00;
    public static final double TAIL_POS_DN      = 0.90;


    // Beam parameters
    public static final double BEAM_POWER       = 0.70;
    public static final int    BEAM_TARGET_IN   = +5000;         // encoder clicks for full retraction
    public static final int    BEAM_TARGET_OUT  = -5000;        // encoder clicks for full extension
    public static final DcMotor.Direction BEAM_DIRECTION = DcMotor.Direction.FORWARD;
    public static final double BEAM_CLAW_OPENED = 2.00;
    public static final double BEAM_CLAW_CLOSED =-2.00;
    public static final double BEAM_SWIVEL_UP   =-2.00;
    public static final double BEAM_SWIVEL_DOWN = 2.00;

    // Misc. motor parameters
    public static final int    MOTOR_TARGET_TOLERANCE =   5;    // encoder clicks
    public static final double MOTOR_LAG_SEC          =   0.25; // seconds
    public static final long   MOTOR_LAG_MILLI        = 250;    // milliseconds


    // Playing field surface constants
    public static final double BALANCING_STONE  = 65;           // encoder clicks per linear inch
    public static final double PLAYING_FIELD    = 75;           // encoder clicks per linear inch
    public static final double ROTATION_RATE    = 10.0;         // encoder clicks per degree of rotation


    // Misc. playing field parameters
    public static final double SHORT  = 36;
    public static final double MEDIUM = 48;
}
