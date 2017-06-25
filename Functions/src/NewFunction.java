
public class NewFunction {

	public static void main( String[] args){
		int[] x = {1,2,3};
		System.out.println(sum(x));

		int[] y = {7,8,9};
		System.out.println(sum(y));

		int[] z = null;
		System.out.println(sum(z));
	}
	
	public static int sum(int[] a) {
		int s = 0;
		
		for (int i=0;  a != null &&  i < a.length; i++){
			s = s + a[i]; 
		}
		
		return s;
	}
}
