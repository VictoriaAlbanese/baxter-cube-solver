
////////////////////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// File name: face_class.cpp
// Purpose: Make a class to represent a Rubik's cube's face
//
////////////////////////////////////////////////////////////////////////////////

#ifndef FACE_CLASS_HPP
#define FACE_CLASS_HPP

#include <iostream>
#include "face_class.hpp"

using namespace std;

////////////////////////////////////////////////////////////////////////////////

// default contructor
// fill the face with one color
Face::Face(int color) {

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			face[i][j] = color;
		}
	}
	
}

// face match contructor
// fills face as speciied
Face::Face(int color_matrix[3][3]) {

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			face[i][j] = color_matrix[i][j];
		}
	}
	
}

// rotate the face to the left
void Face::rotate_left() {}

// rotate the face to the right
void Face::rotate_right() {}

// print the face
void Face::print_face() {

	cout << endl;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {

			switch (this->face[i][j]) {

				case RED:
					cout << "\033[41;1m   \033[0m ";
					break;

				case ORANGE:
					cout << "\033[45;1m   \033[0m ";
					break;

				case BLUE:
					cout << "\033[44;1m   \033[0m ";
					break;

				case GREEN:
					cout << "\033[42;1m   \033[0m ";
					break;

				case WHITE:	
					cout << "\033[47;1m   \033[0m ";
					break;

				case YELLOW:
					cout << "\033[43;1m   \033[0m ";
					break;

				default:
					cout << "\033[40;1m   \033[0m ";
					break;

			}

		}

		cout << endl << endl;

	}

	cout << endl;

}
		
////////////////////////////////////////////////////////////////////////////////

#endif // for FACE_CLASS_HPP

