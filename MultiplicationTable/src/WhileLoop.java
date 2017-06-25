
public class WhileLoop {
	
	public static void main(String[] args) {
		int i=0;
		while (i <= 9){
			i++;
			int j=0;
			while (j <= 9) {
				j++;
				if (i*j < 10) {
					System.out.print(i*j + "   | ");
				} else if (i*j < 100) {
					System.out.print(i*j + "  | ");
				} else {
					System.out.print(i*j + " | ");
				}
			}
			System.out.println();
		}
	}
}
