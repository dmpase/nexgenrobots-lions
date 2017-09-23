package typingStrngs;

import java.io.IOException;

public class Strngs {
	public static void main(String[]args){
		byte [] buf = new byte [1000];
		try {
			String word = null;
			int len = System.in.read(buf);
			byte last_byte = 0;
			int start = 0;
			for (int i=0; i < len; i++) {
				if (last_byte <= ' ' && buf[i] <= ' ') {
					// last byte IS white space, this byte IS white space
					// nothing to do
				} else if (last_byte <= ' ' && ' ' < buf[i]) {
					// last byte IS white space, this byte is NOT white space
					// start a word
					start = i;
				} else if (' ' < last_byte && buf[i] <= ' ') {
					// last byte is NOT white space, this byte IS white space
					// end a word
					word = new String(buf, start, i-start);
					System.out.println(word);
				} else if (' ' < last_byte && ' ' < buf[i]) {
					// last byte is NOT white space, this byte is NOT white space
					// continue word
				}
				last_byte = buf[i];
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
