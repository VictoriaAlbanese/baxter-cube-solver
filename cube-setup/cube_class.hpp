
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
#include <vector>
#include "face_class.hpp"

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

	public:

		Cube();
		Cube(vector<Face> faces); 

		
		Face get_front_face() { return this->front_face; }
		Face get_back_face() { return this->back_face; }
		Face get_top_face() { return this->top_face; }
		Face get_bottom_face() { return this->bottom_face; }
		Face get_left_face() { return this->left_face; }
		Face get_right_face() { return this->right_face; }
		
		void set_front_face(Face new_front_face) { this->front_face = new_front_face; }
		void set_back_face(Face new_back_face) { this->back_face = new_back_face; }
		void set_top_face(Face new_top_face) { this->top_face = new_top_face; }
		void set_bottom_face(Face new_bottom_face) { this->bottom_face = new_bottom_face; }
		void set_left_face(Face new_left_face) { this->left_face = new_left_face; }
		void set_right_face(Face new_right_face) { this->right_face = new_right_face; }
		

		void connect_faces();	
		
		void rotate_front_cw(); 
		void rotate_front_ccw(); 
		
		void rotate_cw_adjacent_top(Face face); 
		void rotate_cw_adjacent_bottom(Face face); 

		void rotate_ccw_adjacent_top(Face face); 
		void rotate_ccw_adjacent_bottom(Face face); 

		void rotate_adjacent_left(Face face, vector<int> replacement_row); 
		void rotate_adjacent_right(Face face, vector<int> replacement_row); 
			
		void print_cube(); 

};

////////////////////////////////////////////////////////////////////////////////

#endif // end of CUBE_CLASS_HPP

////////////////////////////////////////////////////////////////////////////////

