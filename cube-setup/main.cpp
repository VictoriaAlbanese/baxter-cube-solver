
#include <iostream>
#include "helper.hpp"
#include "face_class.hpp"
#include "row_class.hpp"

using namespace std;

int main() {

	
	// Face Testing
	
	//Face blank_face;
	//blank_face.print_face();

	//Face red_face(RED);
	//red_face.print_face();
	
	vector< vector<int> > face_color_matrix;
	vector<int> temp_row1;
	vector<int> temp_row2;
	vector<int> temp_row3;
	
	temp_row1.push_back(WHITE);
	temp_row1.push_back(RED);
	temp_row1.push_back(WHITE);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(RED);
	temp_row2.push_back(ORANGE);
	
	temp_row3.push_back(RED);
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);

	Face interesting_face(face_color_matrix);
	interesting_face.print_face();
		
	interesting_face.rotate_cw();
	interesting_face.print_face();
	
	interesting_face.rotate_ccw();
	interesting_face.rotate_ccw();
	interesting_face.print_face();
	
	/*
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
	*/

	// Cube Testing

	


	return 0;

}
