package org.firstinspires.ftc.teamcode;

/**
 * Created by Doug on 11/24/2017.
 */

import com.qualcomm.robotcore.hardware.DcMotor;

public class DcMotorEnc {
    public DcMotor motor   = null;
    public int     target  = 0;
    public double  max_pwr = 1.0;
    public double  scale   = 100;

    public DcMotorEnc(DcMotor m, int t, double p, double s)
    {
        motor   = m;
        target  = t;
        max_pwr = p;
        scale   = s;
    }

    public void update()
    {
        int c = motor.getCurrentPosition();
        double p = (target - c)/scale;
        p = (p < -max_pwr) ? -max_pwr : (max_pwr < p) ? max_pwr : p;
        motor.setPower(p);
    }
}
