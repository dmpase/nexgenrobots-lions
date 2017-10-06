public class Rotate {

    public static double degrees_to_radians(double degrees)
    {
	return Math.PI*degrees/180;
    }

    public static double radians_to_degrees(double radians)
    {
	return radians*180/Math.PI;
    }

    public static Matrix rotate2d(double radians)
    {
	Matrix result = Matrix.empty(2, 2);

	result.data[0][0] =  Math.cos(radians);
	result.data[0][1] = -Math.sin(radians);

	result.data[1][0] =  Math.sin(radians);
	result.data[1][1] =  Math.cos(radians);

	return result;
    }

    public static Matrix rotate3d_x(double radians)
    {
	Matrix result = Matrix.empty(3, 3);

	result.data[0][0] =  1;
	result.data[0][1] =  0;
	result.data[0][2] =  0;

	result.data[1][0] =  0;
	result.data[1][1] =  Math.cos(radians);
	result.data[1][2] = -Math.sin(radians);

	result.data[2][0] =  0;
	result.data[2][1] =  Math.sin(radians);
	result.data[2][2] =  Math.cos(radians);

	return result;
    }

    public static Matrix rotate3d_y(double radians)
    {
	Matrix result = Matrix.empty(3, 3);

	result.data[0][0] =  Math.cos(radians);
	result.data[0][1] =  0;
	result.data[0][2] =  Math.sin(radians);

	result.data[1][0] =  0;
	result.data[1][1] =  1;
	result.data[1][2] =  0;

	result.data[2][0] = -Math.sin(radians);
	result.data[2][1] =  0;
	result.data[2][2] =  Math.cos(radians);

	return result;
    }

    public static Matrix rotate3d_z(double radians)
    {
	Matrix result = Matrix.empty(3, 3);

	result.data[0][0] =  Math.cos(radians);
	result.data[0][1] = -Math.sin(radians);
	result.data[0][2] =  0;

	result.data[1][0] =  Math.sin(radians);
	result.data[1][1] =  Math.cos(radians);
	result.data[1][2] =  0;

	result.data[2][0] =  0;
	result.data[2][1] =  0;
	result.data[2][2] =  1;

	return result;
    }
}
