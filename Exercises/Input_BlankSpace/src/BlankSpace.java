import java.io.IOException;

public class BlankSpace {
	public static void main ( String[] args){
		byte [] buf = new byte [500];
		try {
			int len = System.in.read(buf);
			for (int i = 0; i <= len; i++){
				if (buf[i] == ' '){
					System.out.println(i);
				}
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		
	}
}
