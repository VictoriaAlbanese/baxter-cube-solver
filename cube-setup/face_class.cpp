
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

// rotate the face clockwise
void Face::rotate_cw() {
	this->transpose();
	for (int i = 0; i < 3; i++) {
    		for (int j = 0; j < 1; j++) {
      			swap(this->face[i][j], this->face[i][2-j]);
    		}
	}
}

// rotate the face counterclockwise
void Face::rotate_ccw() {
	this->transpose();
	for (int j = 0; j < 3; j++) {
    		for (int i = 0; i < 1; i++) {
      			swap(this->face[i][j], this->face[i][2-j]);
    		}
	}
}

// transpose face
void Face::transpose() {
	for(int i = 0; i < 3; i++) {
  		for(int j = i; j < 3; j++) {  
    			swap(this->face[i][j], this->face[j][i]);
 		}
	}
}

// swap function
void swap( int * first, int * second) {
	int temp = *first;
	*first = *second;
	*second = temp;
}

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

