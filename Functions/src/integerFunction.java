
public class integerFunction {
	public static void main( String[] args){
		int y = 3;
		System.out.println(factor(y));
	}
	
	public static int factor(int a) {
		int s = 1;
		if (a <= 0){
			s = 1;
		} else {
			s = a * factor (a-1);
		}
		return s;
	}
}
