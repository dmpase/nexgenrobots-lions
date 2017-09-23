
public class Table {

	public static void main(String[] args) {
		for (int i=0; i <= 10; i++){
			for (int j=0; j <= 10; j++) {
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
