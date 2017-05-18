
#include <iostream>
#include "helper.hpp"
#include "face_class.hpp"
#include "row_class.hpp"

using namespace std;

int main() {

	/*
	// Face Testing

	Face blank_face;
	blank_face.print_face();

	Face red_face(RED);
	red_face.print_face();

	int face_color_matrix[3][3] = {{WHITE, RED, WHITE}, {RED, RED, ORANGE}, {RED, YELLOW, YELLOW}};
	Face interesting_face(face_color_matrix);
	interesting_face.print_face();
		
	interesting_face.rotate_cw();
	interesting_face.print_face();

	interesting_face.rotate_cw();
	interesting_face.rotate_cw();
	interesting_face.print_face();
	*/

	// Row Testing

	Row blank_row;
	blank_row.print_row();

	Row red_row(RED);
	red_row.print_row();

	int row_color_matrix[12] = {BLUE, GREEN, RED, YELLOW, YELLOW, RED, BLUE, WHITE, ORANGE, YELLOW, YELLOW, ORANGE};
	Row interesting_row(row_color_matrix);
	interesting_row.print_row();

	interesting_row.turn_cw();
	interesting_row.print_row();
	
	interesting_row.turn_ccw();
	interesting_row.turn_ccw();
	interesting_row.print_row();

	return 0;

}
