
#include <iostream>
#include "face_class.hpp"

int main() {

	Face blank;
	Face red(RED);
	Face orange(ORANGE);
	Face blue(BLUE);
	Face green(GREEN);
	Face white(WHITE);
	Face yellow(YELLOW);

	int color_matrix[3][3] = {{WHITE, RED, WHITE}, {RED, RED, ORANGE}, {RED, YELLOW, YELLOW}};
	Face interesting(color_matrix);
		
	interesting.print_face();
	interesting.rotate_cw();
	interesting.print_face();

	interesting.rotate_cw();
	interesting.rotate_cw();
	interesting.print_face();

	/*
	interesting.rotate_left();
	interesting.print_face();

	interesting.rotate_right();
	interesting.print_face();

	interesting.rotate_right();
	interesting.print_face();
	*/

	return 0;

}
