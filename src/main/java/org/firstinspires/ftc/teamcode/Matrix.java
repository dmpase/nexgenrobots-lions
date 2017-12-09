
/*******************************************************************************
 * Copyright (c) 1999 - 2017 Douglas M. Pase                                   *
 * All rights reserved. This program and the accompanying materials            *
 * are made available under the terms of the Common Public License v1.0        *
 * which accompanies this distribution, and is available at                    *
 * http://www.opensource.org/licenses/cpl1.0.php                               *
 *                                                                             *
 * Contributors:                                                               *
 *    Douglas M. Pase - initial API and implementation                         *
 ******************************************************************************/

package org.firstinspires.ftc.teamcode;

/******************************************************************************
 *                                                                            *
 * Matrix                                                                     *
 *                                                                            *
 * Author:  Douglas M. Pase                                                   *
 *                                                                            *
 * Date:    November 30, 1999                                                 *
 *                                                                            *
 ******************************************************************************/

public class Matrix {

 /****************************************************************************\
 *                                                                            *
 * Available functions:                                                       *
 *                                                                            *
 *  static double[][] copy       ( double[][] a, double[][] b )               *
 *  static double[]   copy       ( double[]   a, double[]   b )               *
 *  static double[][] copy       ( double[][] a )                             *
 *  static double[]   copy       ( double[]   a )                             *
 *                                                                            *
 *  static double[][] append_cols( double[]   a, double[][] b )               *
 *  static double[][] append_cols( double[][] a, double[]   b )               *
 *  static double[][] append_cols( double[][] a, double[][] b )               *
 *  static double[][] append_rows( double[]   a, double[][] b )               *
 *  static double[][] append_rows( double[][] a, double[]   b )               *
 *  static double[][] append_rows( double[][] a, double[][] b )               *
 *                                                                            *
 *  static double[][] inverse    ( double[][] a )                             *
 *  static double     determinant( double[][] a )                             *
 *  static double[][] minor      ( double[][] a, int mi, int mj )             *
 *                                                                            *
 *  static double[]   row        ( double[][] a, int r )                      *
 *  static double[][] swap_rows  ( double[][] a, int r1, int r2 )             *
 *  static double[][] replace_row( double[][] a, int row, double[] v )        *
 *  static double[][] add_rows   ( double[][] a, int r1, int r2, double s )   *
 *  static double[][] times_row  ( double[][] a, int r1, double s )           *
 *  static double[][] reduce     ( double[][] a )                             *
 *  static double[]   solve      ( double[][] a )                             *
 *  static double[][] invert     ( double[][] a )                             *
 *                                                                            *
 *  static double[]   expand     ( double[]   v, int rows )                   *
 *  static double[][] expand     ( double[][] m, int rows, int cols )         *
 *  static double[]   contract   ( double[]   v, int rows )                   *
 *  static double[][] contract   ( double[][] m, int rows, int cols )         *
 *                                                                            *
 *  static double[]   plus       ( double     s,  double[]   v )              *
 *  static double[]   plus       ( double[]   v,  double     s )              *
 *  static double[][] plus       ( double     s,  double[][] m )              *
 *  static double[][] plus       ( double[][] m,  double     s )              *
 *  static double[]   plus       ( double[]   v1, double[]   v2 )             *
 *  static double[][] plus       ( double[][] m1, double[][] m2 )             *
 *                                                                            *
 *  static double[]   minus      ( double     s,  double[]   v )              *
 *  static double[]   minus      ( double[]   v,  double     s )              *
 *  static double[][] minus      ( double     s,  double[][] m )              *
 *  static double[][] minus      ( double[][] m,  double     s )              *
 *  static double[]   minus      ( double[]   v1, double[]   v2 )             *
 *  static double[][] minus      ( double[][] m1, double[][] m2 )             *
 *                                                                            *
 *  static double[]   times      ( double     s,  double[]   v )              *
 *  static double[]   times      ( double[]   v,  double     s )              *
 *  static double[][] times      ( double     s,  double[][] m )              *
 *  static double[][] times      ( double[][] m,  double     s )              *
 *  static double[]   times      ( double[]   v,  double[][] m )              *
 *  static double[]   times      ( double[][] m,  double[]   v )              *
 *  static double[][] times      ( double[][] m1, double[][] m2 )             *
 *                                                                            *
 *  static double[][] power      ( double[][] m, int p )                      *
 *                                                                            *
 *  static double     length     ( double[] v )                               *
 *  static double     distance   ( double[] v1, double[] v2 )                 *
 *  static double     dot        ( double[] v1, double[] v2 )                 *
 *  static double[]   cross      ( double[] v1, double[] v2 )                 *
 *  static double[]   scale      ( double[] v1, double[] v2 )                 *
 *                                                                            *
 *  static double[][] transpose  ( double[][] m )                             *
 *  static double[]   rotate     ( double[] v, double theta, int d1, int d2 ) *
 *  static double[][] rotate_Tp  ( int n, double theta, int d1, int d2 )      *
 *  static double[][] zero       ( int rows, int cols )                       *
 *  static double[][] unit       ( int rows, int cols )                       *
 *                                                                            *
 *  static void       print      ( double[] a )                               *
 *  static void       print      ( double[][] a )                             *
 *                                                                            *
 \****************************************************************************/

	// STILL TO BE IMPLEMENTED:
		// extract row, col, submatrix
		// reshape vectors/matrices


    public static double degrees_to_radians( double degrees )
    {
    	return degrees * Math.PI / 180;
    }
    
    public static double radians_to_degrees( double radians )
    {
    	return radians * 180 / Math.PI;
    }

    /*
     * copy: Copy matrix b into matrix a and return a.
     */
    public static double[][] copy( double[][] a, double[][] b )
    {
    	if (a.length != b.length) return null;
		for (int i=0; i < b.length; i++) {
			if (a[i].length != b[i].length) return null;
		}

		for (int i=0; i < b.length; i++) {
			for (int j=0; j < b[i].length; j++) {
				a[i][j] = b[i][j];
			}
		}

    	return a;
    }

    /*
     * copy: Copy vector b into vector a and return a.
     */
    public static double[] copy( double[] a, double[] b )
    {
	if (a.length != b.length) return null;

	for (int i=0; i < b.length; i++) {
	    a[i] = b[i];
	}
	
	return a;
    }

    /* 
     * copy: create a new matrix a, copy b into a, return a.
     */
    public static double[][] copy( double[][] b )
    {
		double[][] a = new double[ b.length ][];
	
		for (int i=0; i < b.length; i++) {
		    a[i] = new double[ b[i].length ];
	    	for (int j=0; j < b[i].length; j++) {
				a[i][j] = b[i][j];
	    	}
		}
	
		return a;
    }

    /*
     * copy: create a new vector a, copy b into a, return a.
     */
    public static double[] copy( double[] b )
    {
		double[] a = new double[ b.length ];

		for (int i=0; i < b.length; i++) {
	    	a[i] = b[i];
		}
	
		return a;
    }
	
    /*
     * append_cols: create a new matrix r, copy a and b into r, return r.
     */
    public static double[][] append_cols( double[] a, double[][] b )
    {
    	if (a == null || b == null || a.length != b.length) {
		    return null;
		}
	
		double[][] result = new double[ a.length ][ 1 + b[0].length ];

		for (int i=0; i < a.length; i++) {
		    result[i][0] = a[i];
		}

		for (int i=0; i < b.length; i++) {
		    for (int j=0; j < b[0].length; j++) {
				result[i][1+j] = b[i][j];
		    }
		}
	
		return result;
    }

    /*
     * append_cols: create a new matrix r, copy a and b into r, return r.
     */
    public static double[][] append_cols( double[][] a, double[] b )
    {
		if (a == null || b == null || a.length != b.length) {
		    return null;
		}

		double[][] result = new double[ a.length ][ a[0].length + 1 ];

		for (int i=0; i < a.length; i++) {
		    for (int j=0; j < a[0].length; j++) {
				result[i][j] = a[i][j];
	    	}
		}

		for (int i=0; i < b.length; i++) {
	    	result[i][a[0].length] = b[i];
		}
	
		return result;
    }

    /*
     * append_cols: create a new matrix r, copy a and b into r, return r.
     */
    public static double[][] append_cols( double[][] a, double[][] b )
    {
		if (a == null || b == null || a.length != b.length) {
		    return null;
		}

		double[][] result = new double[ a.length ][ a[0].length + b[0].length ];

		for (int i=0; i < a.length; i++) {
		    for (int j=0; j < a[0].length; j++) {
				result[i][j] = a[i][j];
	    	}
		}

		for (int i=0; i < b.length; i++) {
	    	for (int j=0; j < b[0].length; j++) {
				result[i][a[0].length+j] = b[i][j];
	    	}
		}

		return result;
    }

    public static double[][] append_rows( double[] a, double[][] b )
    {
		if (a == null || b == null || a.length != b[0].length) {
		    return null;
		}

		double[][] result = new double[ 1 + b.length ][ a.length ];

		for (int j=0; j < a.length; j++) {
		    result[0][j] = a[j];
		}

		for (int i=0; i < b.length; i++) {
		    for (int j=0; j < b[0].length; j++) {
				result[1+i][j] = b[i][j];
		    }
		}

		return result;
	}

    public static double[][] append_rows( double[][] a, double[] b )
    {
		if (a == null || b == null || a[0].length != b.length) {
		    return null;
		}

		double[][] result = new double[ a.length + 1 ][ a[0].length ];

		for (int i=0; i < a.length; i++) {
		    for (int j=0; j < a[0].length; j++) {
				result[i][j] = a[i][j];
		    }
		}

		for (int j=0; j < b.length; j++) {
		    result[a.length][j] = b[j];
		}

		return result;
    }

    public static double[][] append_rows( double[][] a, double[][] b )
    {
		if (a == null || b == null || a[0].length != b[0].length) {
		    return null;
		}

		double[][] result = new double[ a.length + b.length ][ a[0].length ];

		for (int i=0; i < a.length; i++) {
		    for (int j=0; j < a[0].length; j++) {
				result[i][j] = a[i][j];
		    }
		}

		for (int i=0; i < b.length; i++) {
		    for (int j=0; j < b[0].length; j++) {
				result[a.length+i][j] = b[i][j];
			}
		}

		return result;
    }

    public static double[][] inverse( double[][] a )
    {
		double[][] b = new double[ a.length ][ a[0].length ];
		double det = determinant(a);
		for (int i=0; i < a.length; i++) {
		    for (int j=0; j < a[0].length; j++) {
				double[][] t = minor(a,i,j);
				double e = determinant(t) / det;
				if ((i+j)%2 == 0) {
				    b[j][i] =  e;
				} else {
				    b[j][i] = -e;
				}
		    }
		}

		return b;
    }

    public static double determinant( double[][] a )
    {
		if (a == null || a.length != a[0].length) {
		    return 0;
		} else if (a.length == 1) {
		    return a[0][0];
		}

		int k = 1;
		double det = 0.0;
		for (int i=0; i < a.length; i++) {
		    double[][] m = minor(a,i,0);
		    det += k * a[i][0] * determinant(m);
		    k = k * -1;
		}

		return det;
    }

    public static double[][] minor(double[][] a, int mi, int mj)
    {
		double[][] b = new double[ a.length - 1 ][ a[0].length - 1 ];

		for (int i=0; i < a.length; i++) {
		    for (int j=0; j < a[0].length; j++) {
				if (i < mi) {
				    if (j < mj) {
						b[i][j] = a[i][j];
				    } else if (j != mj) {
						b[i][j-1] = a[i][j];
				    }
				} else if (i != mi) {
				    if (j < mj) {
						b[i-1][j] = a[i][j];
				    } else if (j != mj) {
						b[i-1][j-1] = a[i][j];
				    }
				}
			}
		}

		return b;
    }

    /*
     * row: extract a row
     */
    public static double[] row( double[][] a, int r )
    {
		double[] v = null;

		if (a != null && 0 <= r && r < a.length) {
		    v = new double[a[r].length];

		    for (int i=0; i < v.length; i++) {
				v[i] = a[r][i];
		    }
		}

		return v;
    }

    /*
     * swap_rows: in place row swap
     */
    public static double[][] swap_rows( double[][] a, int r1, int r2 )
    {
		if (a != null && 0 <= r1 && r1 < a.length && 0 <= r2 && r2 < a[0].length && r1 != r2) {
		    for (int i=0; i < a[r1].length; i++) {
				double t = a[r1][i];
				a[r1][i] = a[r2][i];
				a[r2][i] = t;
		    }
		}

		return a;
    }

    /*
     * replace_row: in place row replacement
     */
    public static double[][] replace_row( double[][] a, int row, double[] v )
    {
		if (a != null && 0 <= row && row < a.length && v.length == a[0].length) {
		    for (int i=0; i < a[row].length; i++) {
				a[row][i] = v[i];
		    }
		}

		return a;
    }

    /*
     * times_row: multiply a row by a scalar
     */
    public static double[][] times_row( double[][] a, int row, double s )
    {
		if (a != null && 0 <= row && row < a.length) {
		    for (int i=0; i < a[row].length; i++) {
				a[row][i] *= s;
		    }
		}

		return a;
    }

    /*
     * add_rows: a[r1][*] += s * a[r2][*]
     */
    public static double[][] add_rows( double[][] a, int r1, int r2, double s )
    {
	if (a != null && 0 <= r1 && r1 < a.length && 0 <= r2 && r2 < a.length) {
	    for (int i=0; i < a[r1].length && i < a[r2].length; i++) {
			a[r1][i] += s * a[r2][i];
	    }
	}

		return a;
    }

    /*
     * reduce: reduce a matrix in place to identity form (rows <= cols) and return the reduced matrix
     */
    public static double[][] reduce( double[][] a )
    {
		if (a != null && a.length <= a[0].length) {
		    // rearrange the rows so a[i][i] is as large as possible
		    for (int i=0; i < a.length; i++) {
				for (int j=i+1; j < a.length; j++) {
				    if (Math.abs(a[i][i]) < Math.abs(a[j][i])) {
						swap_rows(a, i, j);
					}
				}
	    	}

		    // create an upper triangular matrix
		    for (int i=0; i < a.length; i++) {
				// put a 1 on the diagonal element
				times_row(a, i, 1.0/a[i][i]);

				// put a 0 in the rest of this column
				for (int j=i+1; j < a.length; j++) {
				    add_rows(a, j, i, -a[j][i]/a[i][i]);
				}
		    }

	    	// back-solve for the diagonal elements
			for (int i=a.length-1; 0 <= i; i--) {
				for (int j=i-1; 0 <= j; j--) {
				    add_rows(a, j, i, -a[j][i]/a[i][i]);
				}
		    }
		} else {
		    a = null;
		}

		return a;
    }

    /*
     * solve: reduce a matrix to identity form (cols == rows+1) and return the final column
     */
    public static double[] solve( double[][] a )
    {
		double[] v = null;

		if (a != null && a.length+1 == a[0].length) {
		    // create a copy so we don't destroy the original
		    a = copy(a);
		    a = reduce(a);

		    v = new double[a.length];
			for (int i=0; i < a.length; i++) {
				v[i] = a[i][a[i].length-1];
		    }
		}

		return v;
    }

    static double[][] invert( double[][] a )
    {
		if (a != null && a.length == a[0].length) {
		    double[][] c = new double[a.length][2*a.length];

		    // copy a into c (on the left)
		    for (int i=0; i < a.length; i++) {
				for (int j=0; j < a[i].length; j++) {
				    c[i][j] = a[i][j];
				}
		    }

		    // copy I into c (on the right)
		    for (int i=0; i < a.length; i++) {
				for (int j=a[i].length; j < c[i].length; j++) {
				    c[i][j] = 0;
				}
				c[i][a[i].length+i] = 1;
		    }

		    // reduce c into identity form
		    c = reduce(c);

		    // return the right half
		    a = new double[a.length][a.length];

		    for (int i=0; i < a.length; i++) {
				for (int j=0; j < a[i].length; j++) {
				    a[i][j] = c[i][a.length+j];
				}
		    }
		} else {
		    a = null;
		}

		return a;
    }

				// vector expansion
    public static double[] expand( double[] v, int rows )
    {
		if (rows < 0) {
		    return null;
		} else if (v == null && rows == 0) {
		    return null;
		} else if (v == null) {
		    return new double[ rows ];
		}

		double[] result = new double [ v.length + rows ];

		for (int i=0; i < v.length; i++) {
		    result[i] = v[i];
		}

		return result;
    }

				// matrix expansion
    public static double[][] expand( double[][] m, int rows, int cols )
    {
    	if (rows < 0 || cols < 0) {
		    return null;
    	} else if (m == null && (rows == 0 || cols == 0)) {
		    return null;
    	} else if (m == null) {
		    return new double[ rows ][ cols ];
    	} 
    	
    	double[][] result = new double [ m.length + rows ][ m[0].length + cols];
    	
    	for (int i=0; i < m.length; i++) {
		    for (int j=0; j < m[0].length; j++) {
				result[i][j] = m[i][j];
		    }
    	}

    	return result;
    }

				// vector contraction
    public static double[] contract( double[] v, int rows )
    {
    	if (rows < 0) {
		    return null;
    	} else if (v == null && rows == 0) {
		    return null;
    	} else if (v == null) {
		    return new double[ rows ];
    	} 

    	double[] result = new double [ v.length - rows ];

    	for (int i=0; i < result.length; i++) {
		    result[i] = v[i];
    	}

    	return result;
    }

				// matrix contraction
    public static double[][] contract( double[][] m, int rows, int cols )
    {
    	if (rows < 0 || cols < 0) {
		    return null;
    	} else if (m == null && (rows == 0 || cols == 0)) {
		    return null;
    	} else if (m == null) {
		    return new double[ rows ][ cols ];
    	} 

    	double[][] result = new double [ m.length - rows ][ m[0].length - cols];
    	
    	for (int i=0; i < result.length; i++) {
		    for (int j=0; j < result[0].length; j++) {
				result[i][j] = m[i][j];
		    }
    	}

    	return result;
    }

				// scalar/vector addition
    public static double[] plus( double s, double[] v )
    {
    	if (v == null) {
		    return null;
    	}

    	double[] result = new double [ v.length ];

    	for (int i=0; i < result.length; i++) {
		    result[i] = s + v[i];
    	}

    	return result;
    }

				// vector/scalar addition
    public static double[] plus( double[] v, double s )
    {
    	if (v == null) {
		    return null;
    	}

    	double[] result = new double [ v.length ];

    	for (int i=0; i < result.length; i++) {
		    result[i] = v[i] + s;
    	}

    	return result;
    }


				// scalar/dense matrix addition
    public static double[][] plus( double s, double[][] m )
    {
    	if (m == null) {
		    return null;
    	}

    	double[][] result = new double [ m.length ][ m[0].length ];

    	for (int i=0; i < result.length; i++) {
		    for (int j=0; j < result[0].length; j++) {
				result[i][j] = s + m[i][j];
		    }
    	}

    	return result;
    }

				// dense matrix/scalar addition
    public static double[][] plus( double[][] m, double s )
    {
    	if (m == null) {
		    return null;
    	}

    	double[][] result = new double [ m.length ][ m[0].length ];

    	for (int i=0; i < result.length; i++) {
		    for (int j=0; j < result[0].length; j++) {
				result[i][j] = m[i][j] + s;
		    }
    	}

    	return result;
    }

				// vector/vector addition
    public static double[] plus( double[] v1, double[] v2 )
    {
    	if (v1 == null || v2 == null || v1.length != v2.length) {
		    return null;
    	}

    	double[] result = new double [ v1.length ];

    	for (int i=0; i < result.length; i++) {
		    result[i] = v1[i] + v2[i];
    	}

    	return result;
    }

				// dense matrix/matrix addition
    public static double[][] plus( double[][] m1, double[][] m2 )
    {
    	if (m1 == null || m2 == null) {
		    return null;
    	}

    	int m1rows = m1.length;		// matrix 1 rows
    	int m1cols = m1[0].length;	// matrix 1 columns
    	int m2rows = m2.length;		// matrix 2 rows
    	int m2cols = m2[0].length;	// matrix 2 columns

    	if (m1rows != m2rows || m1cols != m2cols) {
    		return null;
    	}

    	double[][] result = new double [ m1rows ][ m1cols ];

    	for (int i=0; i < m1rows; i++) {
		    for (int j=0; j < m1cols; j++) {
				result[i][j] = m1[i][j] + m2[i][j];
		    }
    	}

    	return result;
    }


				// scalar/vector subtraction
    public static double[] minus( double s, double[] v )
    {
    	if (v == null) {
		    return null;
    	}

    	double[] result = new double [ v.length ];

    	for (int i=0; i < result.length; i++) {
		    result[i] = s - v[i];
    	}

    	return result;
    }

				// vector/scalar subtraction
    public static double[] minus( double[] v, double s )
    {
    	if (v == null) {
		    return null;
    	}

    	double[] result = new double [ v.length ];

    	for (int i=0; i < result.length; i++) {
		    result[i] = v[i] - s;
    	}

    	return result;
    }


				// scalar/dense matrix subtraction
    public static double[][] minus( double s, double[][] m )
    {
		if (m == null) {
		    return null;
    	}

    	double[][] result = new double [ m.length ][ m[0].length ];

    	for (int i=0; i < result.length; i++) {
		    for (int j=0; j < result[0].length; j++) {
				result[i][j] = s - m[i][j];
		    }
    	}

    	return result;
    }

				// dense matrix/scalar subtraction
    public static double[][] minus( double[][] m, double s )
    {
    	if (m == null) {
		    return null;
    	}

    	double[][] result = new double [ m.length ][ m[0].length ];

    	for (int i=0; i < result.length; i++) {
		    for (int j=0; j < result[0].length; j++) {
				result[i][j] = m[i][j] - s;
		    }
    	}

    	return result;
    }

				// vector/vector subtraction
    public static double[] minus( double[] v1, double[] v2 )
    {
    	if (v1 == null || v2 == null || v1.length != v2.length) {
		    return null;
    	}

    	double[] result = new double [ v1.length ];

    	for (int i=0; i < result.length; i++) {
		    result[i] = v1[i] - v2[i];
    	}

    	return result;
    }

				// dense matrix/matrix subtraction
    public static double[][] minus( double[][] m1, double[][] m2 )
    {
    	if (m1 == null || m2 == null) {
		    return null;
    	}

    	int m1rows = m1.length;		// matrix 1 rows
    	int m1cols = m1[0].length;	// matrix 1 columns
    	int m2rows = m2.length;		// matrix 2 rows
    	int m2cols = m2[0].length;	// matrix 2 columns
    	
    	if (m1rows != m2rows || m1cols != m2cols) {
		    return null;
    	}

    	double[][] result = new double [ m1rows ][ m1cols ];

    	for (int i=0; i < m1rows; i++) {
		    for (int j=0; j < m1cols; j++) {
				result[i][j] = m1[i][j] - m2[i][j];
		    }
    	}

    	return result;
    } 

				// in place scalar/vector multiplication
    public static double[] times( double s, double[] v )
    {
    	if (v == null) {
		    return null;
    	}

    	double[] result = new double [ v.length ];

    	for (int i=0; i < result.length; i++) {
		    result[i] = s * v[i];
    	}
    	
    	return result;
    }

				// in place vector/scalar multiplication
    public static double[] times( double[] v, double s )
    {
    	if (v == null) {
		    return null;
    	}

    	double[] result = new double [ v.length ];

    	for (int i=0; i < result.length; i++) {
		    result[i] = v[i] * s;
    	}

    	return result;
    }

				// in place scalar/dense matrix multiplication
    public static double[][] times( double s, double[][] m )
    {
    	if (m == null) {
		    return null;
    	}

    	double[][] result = new double [ m.length ][ m[0].length ];

    	for (int i=0; i < result.length; i++) {
		    for (int j=0; j < result[0].length; j++) {
				result[i][j] = s * m[i][j];
		    }
    	}

    	return result;
    }

				// in place dense matrix/scalar multiplication
    public static double[][] times( double[][] m, double s )
    {
    	if (m == null) {
		    return null;
    	}

    	double[][] result = new double [ m.length ][ m[0].length ];

    	for (int i=0; i < result.length; i++) {
		    for (int j=0; j < result[0].length; j++) {
				result[i][j] = m[i][j] * s;
		    }
    	}

    	return result;
    }

				// length of a vector
    public static double distance( double[] v )
    {
    	if (v == null) {
		    return 0;
    	}

    	double result = 0;

    	for (int i=0; i < v.length; i++) {
		    result += v[i] * v[i];
    	}
    	result = Math.sqrt( result );
    	
    	return result;
    }

				// distance between two vectors
    public static double distance( double[] v1, double[] v2 )
    {
    	if (v1 == null || v2 == null || v1.length != v2.length) {
		    return 0;
    	}

    	double result = 0;

    	for (int i=0; i < v1.length; i++) {
		    result += (v1[i] - v2[i]) * (v1[i] - v2[i]);
    	}
    	result = Math.sqrt( result );

    	return result;
    }

				// vector/vector dot product
    public static double dot( double[] v1, double[] v2 )
    {
		if (v1 == null || v2 == null || v1.length != v2.length) {
		    return 0;
    	}
    	
    	double result = 0;

    	for (int i=0; i < v1.length; i++) {
		    result += v1[i] * v2[i];
    	}

    	return result;
    }

				// 3d vector/vector cross product
				// only works for 3 dimensions
    public static double[] cross( double[] v1, double[] v2 )
    {
    	if (v1 == null || v2 == null || v1.length != 3 || v2.length != 3) {
		    return null;
    	}

    	double[] result = new double [ 3 ];

    	result[0] = v1[1] * v2[2] - v1[2] * v2[1];
    	result[1] = v1[2] * v2[0] - v1[0] * v2[2];
    	result[2] = v1[0] * v2[1] - v1[1] * v2[0];
    	
    	return result;
    }

				// vector/vector pairwise multiplication
    public static double[] scale( double[] v1, double[] v2 )
    {
    	if (v1 == null || v2 == null || v1.length != v2.length) {
		    return null;
    	}

    	double[] result = new double [ v1.length ];

    	for (int i=0; i < result.length; i++) {
		    result[i] = v1[i] * v2[i];
    	}

    	return result;
    }

				// vector/dense matrix multiplication
    public static double[] times( double[] v, double[][] m )
    {
    	if (v == null || m == null) {
		    return null;
    	}

    	int vcols = v.length;		// vector columns
    	int mrows = m.length;		// matrix rows
    	int mcols = m[0].length;	// matrix columns

    	if (vcols != mrows) {
		    return null;
    	}

    	double[] result = new double[ mcols ];

    	for (int i=0; i < mcols; i++) {
		    result[i] = 0.0;
			for (int j=0; j < mrows; j++) {
				result[i] += v[j] * m[j][i];
		    }
    	}

    	return result;
    }

				// dense matrix/vector multiplication
    public static double[] times( double[][] m, double[] v )
    {
    	if (v == null || m == null) {
		    return null;
    	}

    	int vrows = v.length;		// vector rows
    	int mrows = m.length;		// matrix rows
    	int mcols = m[0].length;	// matrix columns

    	if (mcols != vrows) {
    		return null;
    	}

    	double[] result = new double[ mrows ];

    	for (int i=0; i < mrows; i++) {
		    result[i] = 0.0;
		    for (int j=0; j < mcols; j++) {
				result[i] += m[i][j] * v[j];
		    }
    	}

    	return result;
    }

				// dense matrix/matrix multiplication
    public static double[][] times( double[][] m1, double[][] m2 )
    {
    	if (m1 == null || m2 == null) {
		    return null;
    	}

    	int m1rows = m1.length;		// matrix 1 rows
    	int m1cols = m1[0].length;	// matrix 1 columns
    	int m2rows = m2.length;		// matrix 2 rows
    	int m2cols = m2[0].length;	// matrix 2 columns

    	if (m1cols != m2rows) {
		    return null;
    	}

    	double[][] result = new double[ m1rows ][ m2cols ];

    	for (int i=0; i < m1rows; i++) {
		    for (int j=0; j < m2cols; j++) {
				result[i][j] = 0.0;
				for (int k=0; k < m1cols; k++) {
				    result[i][j] += m1[i][k] * m2[k][j];
				}
		    }
    	}

    	return result;
    }

				// raise a dense matrix to an integer power
    public static double[][] power( double[][] m, int p )
    {
    	if (m == null || m.length != m[0].length || p < 0) {
		    return null;
    	}

    	double[][] square = m;
    	double[][] result = unit( m.length, m.length );
    	for (int i=p; i > 0; i/=2) {
		    if (i%2 == 1) {
				result = times(square,result);
		    }
		    if (i > 1) {
				square = times(square,square);
		    }
    	}

    	return result;
    }

				// dense matrix transposition
    public static double[][] transpose( double[][] m )
    {
    	if (m == null) {
		    return null;
    	} 

    	int mrows = m.length;		// matrix rows
    	int mcols = m[0].length;	// matrix columns

    	double[][] result = new double[ mcols ][ mrows ];

    	for (int i=0; i < mrows; i++) {
		    for (int j=0; j < mcols; j++) {
				result[j][i] = m[i][j];
		    }
    	}

    	return result;
    }

				// rotate the vector v by theta radians
				// in the d1 * d2 dimensions
				//
				// for example, v = [ .25, .75 ],
				// theta = PI/4, d1 = 0, d2 = 1 means:
				// rotate v in the xy plane by theta.
				// (theta is measured from x toward y,
				// or the observer at the origin rotates
				// from y toward x.)
    public static double[] rotate( double[] v, double theta, int d1, int d2 )
    {
				// ensure v is reasonably valid
    	if (v == null) {
		    return null;
    	}

    	double t[][] = rotate_Tp( v.length, theta, d1, d2 );
    	if (t == null) {
		    return null;
    	}
    	
    	return times( t, v );
    }


				// Give the rotational transformation
				// matrix assuming the operation takes
				// the form T*[x,y,z] where T is the
				// transform matrix, and [x,y,z] is the
				// positional vector.  Rotation is always
				// around the origin.
				//
				// To use the form [x,y,z]*T, transpose
				// the result matrix T.
				//
				// In rotate_Tp:
				//   n is the number of dimensions in [x,...,z]
				//   theta is the angle of rotation
				//   rotation is away from axis d1
				//   rotation is towards axis d2
    public static double[][] rotate_Tp( int n, double theta, int d1, int d2 )
    {
				// ensure d1 and d2 are valid dimensions
    	if (n < 2 || d1 < 0 || n <= d1 || d2 < 0 || n <= d2) {
		    return null;
    	}

    	double t[][] = new double[ n ][ n ];
    	for (int i=0; i < t.length; i++) {
		    for (int j=0; j < t.length; j++) {
				t[i][j] = 0;
		    }
		    t[i][i] = 1;
		}

    	double sin_theta, cos_theta;
    	if (theta == 0 || theta == 2*Math.PI) {
		    cos_theta = 1;
		    sin_theta = 0;
		} else if (theta == Math.PI/2 || theta == -3*Math.PI/2) {
		    cos_theta = 0;
		    sin_theta = 1;
    	} else if (theta == Math.PI || theta == -Math.PI) {
		    cos_theta = -1;
		    sin_theta = 0;
    	} else if (theta == 3*Math.PI/2 || theta == -Math.PI/2) {
		    cos_theta = 0;
		    sin_theta = -1;
    	} else {
		    cos_theta = Math.cos( theta );
		    sin_theta = Math.sin( theta );
    	}

    	if (d1 != d2) {
		    t[ d1 ][ d1 ] =   cos_theta;
		    t[ d1 ][ d2 ] = - sin_theta;
		    t[ d2 ][ d1 ] =   sin_theta;
		    t[ d2 ][ d2 ] =   cos_theta;
    	}
    	
    	return t;
    }


				// Give the translational transformation
				// matrix assuming the operation takes
				// the form T*[x,y,z] where T is the
				// transform matrix, and [x,y,z] is the
				// positional vector.
				//
				// To use the form [x,y,z]*T, transpose
				// the result matrix T.
				//
				// In translate_Tp:
				//   v is the vector of translation
    public static double[][] translate_Tp( double[] v )
    {
    	int n = v.length;

    	double t[][] = new double[ n ][ n ];
    	for (int i=0; i < t.length; i++) {
		    for (int j=0; j < t.length; j++) {
				t[i][j] = 0;
		    }
		    t[i][i] = 1;
		    t[i][n-1] = v[i];
    	}

    	return t;
    }



				// Give the scaling transformation
				// matrix assuming the operation takes
				// the form T*[x,y,z] where T is the
				// transform matrix, and [x,y,z] is the
				// positional vector.
				//
				// To use the form [x,y,z]*T, transpose
				// the result matrix T.
				//
				// In scale_Tp:
				//   v is the vector of scaling values
    public static double[][] scale_Tp( double[] v )
    {
    	int n = v.length;

    	double t[][] = new double[ n ][ n ];
		for (int i=0; i < t.length; i++) {
		    for (int j=0; j < t.length; j++) {
				t[i][j] = 0;
		    }
		    t[i][i] = v[i];
    	}
    	
    	return t;
    }


    public static double[][] zero(int rows, int cols)
    {
    	if (rows < 1 || cols < 1) {
		    return null;
    	}
    	
    	double[][] result = new double[ rows ][ cols ];

    	return result;
    }

    public static double[][] unit(int rows, int cols)
    {
    	if (rows < 1 || cols < 1) {
		    return null;
    	}

    	double[][] result = new double[ rows ][ cols ];

    	for (int i=0; i < rows; i++) {
		    for (int j=0; j < cols; j++) {
				if (i == j) {
				    result[i][j] = 1;
				} else {
				    result[i][j] = 0;
				}
		    }
    	}

    	return result;
    }


    public static void print( double[] a )
    {
    	if (a == null) {
	    	return;
    	}

    	for (int i=0; i < a.length; i++) {
	    	System.out.printf("%6.2f ", a[i]);
    	}
    	System.out.println( "" );
    }


    public static void print( double[][] a )
    {
    	if (a == null) {
	    	return;
    	}

    	for (int i=0; i < a.length; i++) {
	    	System.out.printf( "%3d: ", i );
	    	for (int j=0; j < a[0].length; j++) {
				System.out.printf("%6.2f ", a[i][j]);
	    	}
	    	System.out.println( "" );
    	}
    }
}
