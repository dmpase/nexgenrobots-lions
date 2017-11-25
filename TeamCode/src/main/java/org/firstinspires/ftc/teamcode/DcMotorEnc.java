package org.firstinspires.ftc.teamcode;

/**
 * Created by Doug on 11/24/2017.
 */

import com.qualcomm.robotcore.hardware.DcMotor;

public class DcMotorEnc {
    public DcMotor motor   = null;
    public int     count   = 0;
    public int     target  = 0;
    public double  min_pwr = 0.01;
    public double  max_pwr = 1.0;
    public double  scale   = 100;
    public double  power   = 0;

    public DcMotorEnc(DcMotor motor, int target, double max_pwr, double min_pwr, double scale)
    {
        this.motor   = motor;
        this.target  = target;
        this.max_pwr = max_pwr;
        this.min_pwr = min_pwr;
        this.scale   = scale;
    }

    public void setTargetPosition(int position)
    {
        target = position;
    }

    public void update()
    {
        int c = motor.getCurrentPosition();
        power = (target - c)/scale;
        power = (power < -max_pwr) ? -max_pwr : (max_pwr < power) ? max_pwr : power;
        power = (-min_pwr < power && power < min_pwr) ? 0 : power;
        DcMotor.RunMode mode = motor.getMode();
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(power);
        motor.setMode(mode);
        count += 1;
    }
}
