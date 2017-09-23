package array_sorter;

import java.io.IOException;
import java.util.Random;

public class sort {
	public static void main(String[]args){
		byte [] buf = new byte [1000];
		try {
			int len = System.in.read(buf);
			String str = new String(buf,0,len);
			str = str.trim();
			int num = Integer.parseInt(str);
			int[] ray = new int [num];
			Random r = new Random();
			for(int i = 0; i < ray.length; i++){
				ray[i] = r.nextInt(100);
			} 
			
			for(int i = 0; i < ray.length; i++){
				for(int j = 1; j < ray.length; j++){
					if (ray[j-1] > ray[j]) {
						int t = ray[j-1];
						ray[j-1] = ray[j];
						ray[j] = t;
					}
				}
			}
			
			for(int i = 0; i < ray.length; i++){
				System.out.println(ray[i]);
			} 
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}
