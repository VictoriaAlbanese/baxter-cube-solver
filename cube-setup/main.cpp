
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

	yellow.print_face();
	blue.print_face();
	red.print_face();
	green.print_face();
	white.print_face();
	orange.print_face();

	return 0;

}
