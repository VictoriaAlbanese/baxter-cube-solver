
////////////////////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// File name: cube_class.hpp
// Purpose: Make a class to represent a Rubik's cube
//
////////////////////////////////////////////////////////////////////////////////

#ifndef CUBE_CLASS_HPP
#define CUBE_CLASS_HPP

#include <iostream>
#include "face_class.hpp"
#include "row_class.hpp"

using namespace std;

////////////////////////////////////////////////////////////////////////////////

// Cube Class

class Cube {

	private:

		Face front_face;
		Face back_face;
		Face top_face;
		Face bottom_face;
		Face left_face;
		Face right_face;

		Row front_row;
		Row halfway_row;
		Row back_row;
		Row top_row;
		Row middle_row;
		Row bottom_row;
		Row left_row;
		Row center_row;
		Row right_row;

	public:

		Cube();
		Cube(Face faces[6]); 
		Cube(Row rows[9]);
		/*
		Face get_front_face() { return this->front_face; }
		Face get_back_face() { return this->back_face; }
		Face get_top_face() { return this->top_face; }
		Face get_bottom_face() { return this->bottom_face; }
		Face get_left_face() { return this->left_face; }
		Face get_right_face() { return this->right_face; }

		Row get_front_row() { return this->front_row; }
		Row get_halfway_row() { return this->halfway_row; }
		Row get_back_row() { return this->back_row; }
		Row get_top_row() { return this->top_row; }
		Row get_middle_row() { return this->middle_row; }
		Row get_bottom_row() { return this->bottom_row; }
		Row get_left_row() { return this->left_row; }
		Row get_center_row() { return this->center_row; }
		Row get_right_row() { return this->right_row; }

		void set_front_face(Face new_front_face) { this->front_face = new_front_face; }
		void set_back_face(Face new_back_face) { this->back_face = new_back_face; }
		void set_top_face(Face new_top_face) { this->top_face = new_top_face; }
		void set_bottom_face(Face new_bottom_face) { this->bottom_face = new_bottom_face }
		void set_left_face(Face new_left_face) { this->left_face = new_left_face; }
		void set_right_face(Face new_right_face) { this->right_face = new_right_face; }

		void set_front_row(Row new_front_row) { this->front_row = new_front_row; }
		void set_halfway_row(Row new_halfway_row) { this->halfway_row = new_halfway_row; }
		void set_back_row(Row new_back_row) { this->back_row = new_back_row; }
		void set_top_row(Row new_top_row) { this->top_row = new_top_row; }
		void set_middle_row(Row new_middle_row) { this->middle_row = new_middle_row; }
		void set_bottom_row(Row new_bottom_row) { this->bottom_row = new_bottom_row; }
		void set_left_row(Row new_left_row) { this->left_row = new_left_row; }
		void set_center_row(Row new_center_row) { this->center_row = new_center_row; }
		void set_right_row(Row new_right_row) { this->right_row = new_right_row; }
		*/
		void faces_to_rows();
		void rows_to_faces();
	
		void print_cube(); 

};

////////////////////////////////////////////////////////////////////////////////

#endif // end of CUBE_CLASS_HPP

////////////////////////////////////////////////////////////////////////////////

