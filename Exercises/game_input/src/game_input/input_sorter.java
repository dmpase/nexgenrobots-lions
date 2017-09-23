package game_input;

import java.io.IOException;

public class input_sorter {
	public static void main(String[]args)
	{
		String[] x = read();
		
		for(int i = 0; i < x.length; i++){
			System.out.println(x[i]);
		}
	}
	
	public static String[] read ()
	{
		byte [] buf = new byte [1000];
		String word = null;
		byte last_byte = 0;
		int start = 0;
		int wordcount = 0;
		String[] tmp = new String [1000];
		try {
			int len = System.in.read(buf);

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
					tmp[wordcount] = word;
					wordcount++;
				} else if (' ' < last_byte && ' ' < buf[i]) {
					// last byte is NOT white space, this byte is NOT white space
					// continue word
				}
				last_byte = buf[i];
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		String[] result = new String [wordcount];
		
		for(int i=0; i < wordcount; i++){
			result[i] = tmp[i];
		}
		
		return result;
	}
}
