
////////////////////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// File name: face_class.hpp
// Purpose: Make a class to represent a Rubik's cube's face
//
////////////////////////////////////////////////////////////////////////////////

#ifndef FACE_CLASS_HPP
#define FACE_CLASS_HPP

#include <iostream>
#include <cstdlib>
#include <vector>
#include <iterator>
#include "helper.hpp"

using std::vector;

////////////////////////////////////////////////////////////////////////////////

// Face Class

class Face {

	private:
		
		vector< vector<int> > face;
		
		vector<int> row1;
		vector<int> row2;
		vector<int> row3;

		vector<int> column1;
		vector<int> column2;
		vector<int> column3;

	public:

		Face(int color = -1);
		Face(vector< vector<int> > color_matrix);

		vector< vector<int> > get_face() { return this->face; }
		void set_face(vector< vector<int> > new_face) { this->face = new_face; }

		void rotate_cw();
		void rotate_ccw();
		
		void print_face();

};

////////////////////////////////////////////////////////////////////////////////

#endif // end of FACE_CLASS_HPP

////////////////////////////////////////////////////////////////////////////////

