
public class DoWhileLoop {
	public static void main(String[] args) {
		int i=0;
		do {
			int j=0;
			do {
				if (i*j < 10) {
					System.out.print(i*j + "   | ");
				} else if (i*j < 100) {
					System.out.print(i*j + "  | ");
				} else {
					System.out.print(i*j + " | ");
				}
				j++;
			} while (j <= 10);
			System.out.println();
			i++;
		} while (i <= 10);
	}
}
