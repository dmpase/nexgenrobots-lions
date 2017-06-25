
public class Fib {
	public static void main( String[] args){
		for (int y = 0; y < 15; y++){
			System.out.print(" " + fib(y));
		}
	}
	
	public static int fib(int a) {
		int s = 0;
		if (a < 1){
			s = 0;
		} else if (a < 2){
			s = 1;
		} else {
			s = fib (a-1) + fib (a-2);
		}
		return s;
	}
}

