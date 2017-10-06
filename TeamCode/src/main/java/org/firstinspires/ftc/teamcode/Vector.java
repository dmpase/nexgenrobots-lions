public class Vector {

    public static void main(String[] args)
    {
	Vector x = Vector.unit(2);
	Matrix transform = Rotate.rotate2d(Rotate.degrees_to_radians(45));
	Vector y = x.multiply(transform);
	Vector z = transform.multiply(x);
	Vector a = transform.multiply(x).multiply(transform);
	x.print();
	transform.print();
	y.print();
	z.print();
	a.print();
    }

    public final double[] data;

    private Vector(int rows)
    {
	data = new double[rows];
    }

    public static Vector empty(int rows)
    {
	Vector result = new Vector(rows);

	for (int i=0; i < result.data.length; i++) {
	    result.data[i] = Double.NaN;
	}

	return result;
    }

    public static Vector zero(int rows)
    {
	Vector result = new Vector(rows);

	for (int i=0; i < result.data.length; i++) {
	    result.data[i] = 0;
	}

	return result;
    }

    public static Vector unit(int rows)
    {
	Vector result = new Vector(rows);

	result.data[0] = 1;
	for (int i=1; i < result.data.length; i++) {
	    result.data[i] = 0;
	}

	return result;
    }

    public Vector add(Vector b)
    {
	Vector result = null;

	if (b != null && data.length == b.data.length) {
	    result = new Vector(data.length);
	    for (int i=0; i < data.length; i++) {
		result.data[i] = data[i] + b.data[i];
	    }
	}

	return result;
    }

    public Vector subtract(Vector b)
    {
	Vector result = null;

	if (b != null && data.length == b.data.length) {
	    result = new Vector(data.length);
	    for (int i=0; i < data.length; i++) {
		result.data[i] = data[i] - b.data[i];
	    }
	}

	return result;
    }

    public double dot(Vector b)
    {
	double result = Double.NaN;

	if (b != null) {
	    final int a_cols = data.length;
	    final int b_rows = b.data.length;

	    if (a_cols == b_rows) {
		result = 0;
		for (int k=0; k < a_cols; k++) {
		    result += data[k] * b.data[k];
		}
	    }
	}

	return result;
    }

    public Vector multiply(Matrix b)
    {
	Vector result = null;

	if (b != null) {
	    final int a_cols = data.length;
	    final int b_rows = b.data.length;
	    final int b_cols = b.data[0].length;

	    if (a_cols == b_rows) {
		result = new Vector(b_cols);
		for (int j=0; j < b_cols; j++) {
		    result.data[j] = 0;
		    for (int k=0; k < a_cols; k++) {
			result.data[j] += data[k] * b.data[k][j];
		    }
		}
	    }
	}

	return result;
    }

    public void print()
    {
	for (int i=0; i < data.length; i++) {
	    System.out.println(data[i]);
	}
    }
}
