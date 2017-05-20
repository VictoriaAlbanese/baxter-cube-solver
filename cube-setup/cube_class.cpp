
////////////////////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// File name: cube_class.hpp
// Purpose: Make a class to represent a Rubik's cube
//
////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include "cube_class.hpp"

using namespace std;

////////////////////////////////////////////////////////////////////////////////

// Constructors

Cube::Cube() {

	Face red_face(RED);
	Face orange_face(ORANGE);
	Face white_face(WHITE);
	Face yellow_face(YELLOW);
	Face green_face(GREEN);
	Face blue_face(BLUE);

	this->front_face = red_face;
	this->back_face = orange_face;
	this->top_face = white_face;
	this->bottom_face = yellow_face;
	this->left_face = green_face;
	this->right_face = blue_face;

	faces_to_rows();

}

Cube::Cube(Face faces[6]) {

	this->front_face = faces[0];
	this->back_face = faces[1];
	this->top_face = faces[2];
	this->bottom_face = faces[3];
	this->left_face = faces[4];
	this->right_face = faces[5];

	faces_to_rows();

} 

Cube::Cube(Row rows[6]) {
	
	this->front_row = rows[0];
	this->halfway_row = rows[1];
	this->back_row = rows[2];
	this->top_row = rows[3];
	this->middle_row = rows[4];
	this->bottom_row = rows[5];
	this->left_row = rows[6];
	this->center_row = rows[7];
	this->right_row = rows[8];

	rows_to_faces();

}

////////////////////////////////////////////////////////////////////////////////

// Conversion Functions

void Cube::faces_to_rows() {

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (i == 0) this->left_face[i][j] = this->top_row[j];	
			if (i == 1) this->left_face[i][j] = this->middle_row[j];	
			if (i == 2) this->left_face[i][j] = this->bottom_row[j];	
		}
	}

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (i == 0) this->front_face[i][j] = this->top_row[j + 3];	
			if (i == 1) this->front_face[i][j] = this->middle_row[j + 3];	
			if (i == 2) this->front_face[i][j] = this->bottom_row[j + 3];	
		}
	}

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (i == 0) this->right_face[i][j] = this->top_row[j + 6];	
			if (i == 1) this->right_face[i][j] = this->middle_row[j + 6];	
			if (i == 2) this->right_face[i][j] = this->bottom_row[j + 6];	
		}
	}

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (i == 0) this->back_face[i][j] = this->top_row[j + 9];	
			if (i == 1) this->back_face[i][j] = this->middle_row[j + 9];	
			if (i == 2) this->back_face[i][j] = this->bottom_row[j + 9];	
		}
	}

	for (int j = 0; j < 3; j++) {
		for (int i = 0; i < 3; i++) {
			if (i == 0) this->top_face[i][j] = this->left_row[j];	
			if (i == 1) this->top_face[i][j] = this->center_row[j];	
			if (i == 2) this->top_face[i][j] = this->right_row[j];	
		}
	}

	for (int j = 0; j < 3; j++) {
		for (int i = 0; i < 3; i++) {
			if (i == 0) this->front_face[i][j] = this->left_row[j + 3];	
			if (i == 1) this->front_face[i][j] = this->center_row[j + 3];	
			if (i == 2) this->front_face[i][j] = this->right_row[j + 3];	
		}
	}

	for (int j = 0; j < 3; j++) {
		for (int i = 0; i < 3; i++) {
			if (i == 0) this->bottom_face[i][j] = this->left_row[j + 6];	
			if (i == 1) this->bottom_face[i][j] = this->center_row[j + 6];	
			if (i == 2) this->bottom_face[i][j] = this->right_row[j + 6];	
		}
	}

	for (int j = 2; j < 0; j--) {
		for (int i = 0; i < 3; i++) {
			if (i == 0) this->back_face[i][j] = this->left_row[j + 9];	
			if (i == 1) this->back_face[i][j] = this->center_row[j + 9];	
			if (i == 2) this->back_face[i][j] = this->right_row[j + 9];	
		}
	}

	for (int j = 2; j < 0; j--) {
		for (int i = 0; i < 3; i++) {
			if (i == 0) this->left_face[i][j] = this->front_row[j];	
			if (i == 1) this->left_face[i][j] = this->halfway_row[j];	
			if (i == 2) this->left_face[i][j] = this->back_row[j];	
		}
	}

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (i == 0) this->top_face[i][j] = this->front_row[j + 3];	
			if (i == 1) this->top_face[i][j] = this->halfway_row[j + 3];	
			if (i == 2) this->top_face[i][j] = this->back_row[j + 3];	
		}
	}

	for (int j = 0; j < 3; j++) {
		for (int i = 0; i < 3; i++) {
			if (i == 0) this->right_face[i][j] = this->front_row[j + 6];	
			if (i == 1) this->right_face[i][j] = this->halfway_row[j + 6];	
			if (i == 2) this->right_face[i][j] = this->back_row[j + 6];	
		}
	}

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (i == 0) this->back_face[i][j] = this->front_row[j + 9];	
			if (i == 1) this->back_face[i][j] = this->halfway_row[j + 9];	
			if (i == 2) this->back_face[i][j] = this->back_row[j + 9];	
		}
	}

}

void Cube::rows_to_faces() {

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (i == 0) this->top_row[j] = left_face[i][j];
			if (i == 1) this->middle_row[j] = left_face[i][j];
			if (i == 2) this->bottom_row[j] = left_face[i][j];
		}
	}

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (i == 0) this->top_row[j + 3] = front_face[i][j];
			if (i == 1) this->middle_row[j + 3] = front_face[i][j];
			if (i == 2) this->bottom_row[j + 3] = front_face[i][j];
		}
	}

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (i == 0) this->top_row[j + 6] = right_face[i][j];
			if (i == 1) this->middle_row[j + 6] = right_face[i][j];
			if (i == 2) this->bottom_row[j + 6] = right_face[i][j];
		}
	}

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (i == 0) this->top_row[j + 9] = back_face[i][j];
			if (i == 1) this->middle_row[j + 9] = back_face[i][j];
			if (i == 2) this->bottom_row[j + 9] = back_face[i][j];
		}
	}

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (i == 0) this->back_row[j + 3] = top_face[i][j];
			if (i == 1) this->halfway_row[j + 3] = top_face[i][j];
			if (i == 2) this->front_row[j + 3] = top_face[i][j];
		}
	}

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			if (i == 0) this->front_row[j + 9] = bottom_face[i][j];
			if (i == 1) this->halfway_row[j + 9] = bottom_face[i][j];
			if (i == 2) this->back_row[j + 9] = bottom_face[i][j];
		}
	}

}

////////////////////////////////////////////////////////////////////////////////

// Handle Printing
	
void Cube::print_cube() {

	cout << endl;

	Row top_top_row({-1, -1, -1, WHITE, WHITE, WHITE, -1, -1, 1});
	Row top_middle_row({-1, -1, -1, WHITE, WHITE, WHITE, -1, -1, -1});
	Row top_bottom_row({-1, -1, -1, WHITE, WHITE, WHITE, -1, -1, -1});

	Row bottom_top_row({-1, -1, -1, YELLOW, YELLOW, YELLOW, -1, -1, -1});
	Row bottom_middle_row({-1, -1, -1, YELLOW, YELLOW, YELLOW, -1, -1, -1});
	Row bottom_bottom_row({-1, -1, -1, YELLOW, YELLOW, YELLOW, -1, -1, -1});

	Row back_top_row({-1, -1, -1, ORANGE, ORANGE, ORANGE, -1, -1, -1});
	Row back_middle_row({-1, -1, -1, ORANGE, ORANGE, ORANGE, -1, -1, -1});
	Row back_bottom_row({-1, -1, -1, ORANGE, ORANGE, ORANGE, -1, -1, -1});

	top_top_row.print_row();
	top_middle_row.print_row();
	top_bottom_row.print_row();

	this->top_row.print_row();
	this->middle_row.print_row();
	this->bottom_row.print_row();

	bottom_top_row.print_row();
	bottom_middle_row.print_row();
	bottom_bottom_row.print_row();

	back_top_row.print_row();
	back_middle_row.print_row();
	back_bottom_row.print_row();

	cout << endl;

}

////////////////////////////////////////////////////////////////////////////////

