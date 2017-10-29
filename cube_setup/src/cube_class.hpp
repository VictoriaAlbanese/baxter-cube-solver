
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
#include <stdexcept>
#include <vector>

#include "face_class.hpp"

#define CW 0
#define CCW 1

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
		void connect_faces();	

	public:
		Cube();
		Cube(vector<Face> faces); 
		bool operator==(const Cube &other) const; 
		bool operator!=(const Cube &other) const; 
		
		Face get_front_face() { return this->front_face; }
		Face get_back_face() { return this->back_face; }
		Face get_top_face() { return this->top_face; }
		Face get_bottom_face() { return this->bottom_face; }
		Face get_left_face() { return this->left_face; }
		Face get_right_face() { return this->right_face; }
		
		void rotate_front_face(int direction); 
		void rotate_back_face(int direction); 
		void rotate_top_face(int direction); 
		void rotate_bottom_face(int direction); 
		void rotate_left_face(int direction); 
		void rotate_right_face(int direction); 

		vector<int> get_column(Face face, int column, bool is_reversed);
		vector< vector<int> > fix_face(Face face, int column, vector<int> replacement, bool is_reversed); 

		void print(); 
};

#endif // end of CUBE_CLASS_HPP

////////////////////////////////////////////////////////////////////////////////

