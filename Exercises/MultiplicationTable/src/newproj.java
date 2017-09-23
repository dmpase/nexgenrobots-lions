
public class newproj {
	public static void main(String[] args) {
		for (int i=1; i <= 10; i++){
			for (int j=1; j <= 10; j++) {
				if (i*j % 5 >= 1) {
					System.out.print("   |" );
				} else if (i*j <= 5) {
					System.out.print(i*j + "  |");
				} else if (i*j >= 100) {
					System.out.print(i*j + "|");
				} else {
					System.out.print(i*j + " |");
				}
			}
			System.out.println("");
		}
	}
}


