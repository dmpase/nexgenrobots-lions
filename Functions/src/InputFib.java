import java.io.IOException;

public class InputFib {
	public static void main( String[] args){
		System.out.print("type a number: ");
		byte [] buf = new byte [500];
		int len = 0;
		try {
			len = System.in.read(buf);
		} catch (IOException e) {
			e.printStackTrace();
		}
		String str = new String(buf, 0, len-2);
		int x = Integer.parseInt(str);
		int y = fib(x);
		System.out.println("The fibbonachi of " + x + " is " + y);
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
