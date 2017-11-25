package org.firstinspires.ftc.teamcode;

/**
 * Created by Doug on 11/25/2017.
 */

public class Distance {
    public double  coefficient = 1;
    public double  power       = 1;
    public double  offset      = 0;

    enum DistanceUnit {CM, IN, M, FT}

    public Distance(double coefficient, double power, double offset)
    {
        this.coefficient = coefficient;
        this.power       = power;
        this.offset      = offset;
    }

    public double distance(double voltage)
    {
        return coefficient * Math.pow(voltage, power) + offset;
    }

    public double convert(double value, DistanceUnit from, DistanceUnit to)
    {
        if (from == DistanceUnit.CM && to == DistanceUnit.IN) {
            return value * 0.393701;
        } else if (from == DistanceUnit.CM && to == DistanceUnit.M) {
            return value * 0.01;
        } else if (from == DistanceUnit.CM && to == DistanceUnit.FT) {
            return value * 0.0328084;
        } else if (from == DistanceUnit.IN && to == DistanceUnit.CM) {
            return value * 2.54;
        } else if (from == DistanceUnit.IN && to == DistanceUnit.M) {
            return value * 0.0254;
        } else if (from == DistanceUnit.IN && to == DistanceUnit.FT) {
            return value / 12;
        } else if (from == DistanceUnit.M && to == DistanceUnit.CM) {
            return value * 100;
        } else if (from == DistanceUnit.M && to == DistanceUnit.IN) {
            return value * 39.3700629921;
        } else if (from == DistanceUnit.M && to == DistanceUnit.FT) {
            return value * 3.2808385826749999481;
        } else if (from == DistanceUnit.FT && to == DistanceUnit.CM) {
            return value * 30.48;
        } else if (from == DistanceUnit.FT && to == DistanceUnit.IN) {
            return value * 12;
        } else if (from == DistanceUnit.FT && to == DistanceUnit.M) {
            return value * 0.3048;
        }

        return value;
    }
}
