
public class Functionz {
	public static void main( String[] args){
		int[] x = {1,2,3};
		print_array(x);
	}
	public static void print_array(int[] a){
		for (int i=0; a != null && i < a.length; i++){
			System.out.println(a[i]);
		}
	}
}
