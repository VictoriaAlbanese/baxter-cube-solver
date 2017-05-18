
////////////////////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// File name: face_class.cpp
// Purpose: Make a class to represent a Rubik's cube's face
//
////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include "face_class.hpp"

using namespace std;

////////////////////////////////////////////////////////////////////////////////

// Constructors

Face::Face(int color) {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			face[i][j] = color;
		}
	}
}

Face::Face(int color_matrix[3][3]) {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			face[i][j] = color_matrix[i][j];
		}
	}
}

////////////////////////////////////////////////////////////////////////////////

// Rotating Functions

void Face::rotate_cw() {
	this->transpose();
	for (int i = 0; i < 3; i++) {
      		swap(&this->face[i][0], &this->face[i][2]);
	}
}

void Face::rotate_ccw() {
	this->transpose();
	for (int j = 0; j < 3; j++) {
      		swap(&this->face[0][j], &this->face[0][2-j]);
	}
}

void Face::transpose() {
	for(int i = 0; i < 3; i++) {
  		for(int j = i; j < 3; j++) {  
    			swap(&this->face[i][j], &this->face[j][i]);
 		}
	}
}
		
////////////////////////////////////////////////////////////////////////////////

// Handle Printing

void Face::print_face() {
	cout << endl;
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			print_square(this->face[i][j]);
		}
		cout << endl << endl;
	}
	cout << endl;
}
		
////////////////////////////////////////////////////////////////////////////////

