
////////////////////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// File name: face_class.hpp
// Purpose: Declare a class to represent a Rubik's cube's face
//
////////////////////////////////////////////////////////////////////////////////

#ifndef FACE_CLASS_HPP
#define FACE_CLASS_HPP

#include <cstddef>
#include <iostream>
#include <iterator>
#include <stdexcept>
#include <vector>

#include "helper.hpp"

using std::invalid_argument;
using std::vector;

////////////////////////////////////////////////////////////////////////////////

// Face Class

class Face {

	private:
		vector< vector<int> > face;
		vector<int> row1;
		vector<int> row2;
		vector<int> row3;
		Face * adjacent_top;
		Face * adjacent_bottom;
		Face * adjacent_left;
		Face * adjacent_right;

	public:
		Face();
		Face(vector< vector<int> > color_matrix);
		bool operator==(const Face &other) const; 
		bool operator!=(const Face &other) const; 
		
		vector< vector<int> > get_face() { return this->face; }
		vector<int> get_row1() { return this->row1; }
		vector<int> get_row2() { return this->row2; }
		vector<int> get_row3() { return this->row3; }
		Face * get_adjacent_top() { return this->adjacent_top; }
		Face * get_adjacent_bottom() { return this->adjacent_bottom; }
		Face * get_adjacent_left() { return this->adjacent_left; }
		Face * get_adjacent_right() { return this->adjacent_right; }
		void set_face(vector< vector<int> > new_face); 
		void set_row1(vector<int> new_row);
		void set_row2(vector<int> new_row);
		void set_row3(vector<int> new_row);
		void set_adjacent_top(Face * new_face) { this->adjacent_top = new_face; }
		void set_adjacent_bottom(Face * new_face) { this->adjacent_bottom = new_face; }
		void set_adjacent_left(Face * new_face) { this->adjacent_left = new_face; }
		void set_adjacent_right(Face * new_face) { this->adjacent_right = new_face; }

		void rotate_cw();
		void rotate_ccw();
		
		void print_face();

};

#endif // end of FACE_CLASS_HPP

////////////////////////////////////////////////////////////////////////////////

