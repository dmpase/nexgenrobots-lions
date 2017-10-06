public class Matrix {

    public final double[][] data;

    private Matrix(int rows, int cols)
    {
	data = new double[rows][cols];
    }

    public static Matrix empty(int rows, int cols)
    {
	Matrix result = new Matrix(rows, cols);

	for (int i=0; i < result.data.length; i++) {
	    for (int j=0; j < result.data[i].length; j++) {
		result.data[i][j] = Double.NaN;
	    }
	}

	return result;
    }

    public static Matrix zero(int rows, int cols)
    {
	Matrix result = new Matrix(rows, cols);

	for (int i=0; i < result.data.length; i++) {
	    for (int j=0; j < result.data[i].length; j++) {
		result.data[i][j] = 0;
	    }
	}

	return result;
    }

    public static Matrix unit(int rows, int cols)
    {
	Matrix result = new Matrix(rows, cols);

	for (int i=0; i < result.data.length; i++) {
	    for (int j=0; j < result.data[i].length; j++) {
		result.data[i][j] = 0;
	    }

	    if (i < result.data[i].length) {
		result.data[i][i] = 1;
	    }
	}

	return result;
    }

    public Matrix add(Matrix b)
    {
	Matrix result = null;

	if (b != null && data.length == b.data.length && data[0].length == b.data[0].length) {
	    result = new Matrix(data.length, data[0].length);
	    for (int i=0; i < data.length; i++) {
		for (int j=0; j < data[i].length; j++) {
		    result.data[i][j] = data[i][j] + b.data[i][j];
		}
	    }
	}

	return result;
    }

    public Matrix subtract(Matrix b)
    {
	Matrix result = null;

	if (b != null && data.length == b.data.length && data[0].length == b.data[0].length) {
	    result = new Matrix(data.length, data[0].length);
	    for (int i=0; i < data.length; i++) {
		for (int j=0; j < data[i].length; j++) {
		    result.data[i][j] = data[i][j] - b.data[i][j];
		}
	    }
	}

	return result;
    }

    public Vector multiply(Vector b)
    {
	Vector result = null;

	if (b != null) {
	    final int a_rows = data.length;
	    final int a_cols = data[0].length;
	    final int b_rows = b.data.length;

	    if (a_cols == b_rows) {
		result = Vector.empty(a_rows);
		for (int i=0; i < a_rows; i++) {
		    result.data[i] = 0;
		    for (int k=0; k < a_cols; k++) {
			result.data[i] += data[i][k] * b.data[k];
		    }
		}
	    }
	}

	return result;
    }

    public Matrix multiply(Matrix b)
    {
	Matrix result = null;

	if (b != null) {
	    final int a_rows = data.length;
	    final int a_cols = data[0].length;
	    final int b_rows = b.data.length;
	    final int b_cols = b.data[0].length;

	    if (a_cols == b_rows) {
		result = new Matrix(a_rows, b_cols);
		for (int i=0; i < a_rows; i++) {			// row of a[i][*]
		    for (int j=0; j < b_cols; j++) {			// col of b[*][j]
			result.data[i][j] = 0;
			for (int k=0; k < a_cols; k++) {		// c[i][j] = sum(a[i][*] x b[*][j])
			    result.data[i][j] += data[i][k] * b.data[k][j];
			}
		    }
		}
	    }
	}

	return result;
    }

    public void print()
    {
	for (int i=0; i < data.length; i++) {
	    for (int j=0; j < data[i].length; j++) {
		System.out.print(data[i][j] + " ");
	    }
	    System.out.println();
	}
    }
}
