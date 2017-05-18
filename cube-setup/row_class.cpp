
////////////////////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// File name: row_class.cpp
// Purpose: Make a class to represent a Rubik's cube's rows 
//
////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include "row_class.hpp"

using namespace std;

////////////////////////////////////////////////////////////////////////////////

// Constructors

Row::Row(int color) {
	this->start_index = 0;
	for (int i = 0; i < 12; i++) {
		this->row[i] = color;
	}
}


Row::Row(int color_matrix[12]) {
	this->start_index = 0;
	for (int i = 0; i < 12; i++) {
		this->row[i] = color_matrix[i];
	}
}

////////////////////////////////////////////////////////////////////////////////

// Turn Functions

void Row::turn_cw() {
	start_index = ( start_index + 3 ) % 12;
}

void Row::turn_ccw() {
	start_index = ( start_index - 3 ) % 12;
	if (start_index < 0) {
		start_index += 12;
	}
}

////////////////////////////////////////////////////////////////////////////////

// Handle Printing

void Row::print_row() {
		
	for (int i = start_index; i < 12; i++) {
		print_square(this->row[i]);
	}

	for (int i = 0; i < start_index; i++) {
		print_square(this->row[i]);
	}
	cout << endl << endl;
}

////////////////////////////////////////////////////////////////////////////////

