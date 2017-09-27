
////////////////////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// File name: cube_class.hpp
// Purpose: Make a class to represent a Rubik's cube
//
////////////////////////////////////////////////////////////////////////////////

#include "cube_class.hpp"

using namespace std;

////////////////////////////////////////////////////////////////////////////////

// Constructors

Cube::Cube() 
{
	Face blank_face;	

	this->front_face = blank_face;
	this->back_face = blank_face;
	this->top_face = blank_face;
	this->bottom_face = blank_face;
	this->left_face = blank_face;
	this->right_face = blank_face;
}

Cube::Cube(vector<Face> faces) 
{
	if (faces.size() != 6) {
		cout << "Invalid number of faces: expected 6" << endl;
		exit(1);
	}

	this->front_face = faces[0];
	this->back_face = faces[1];
	this->top_face = faces[2];
	this->bottom_face = faces[3];
	this->left_face = faces[4];
	this->right_face = faces[5];
} 

////////////////////////////////////////////////////////////////////////////////

// Connect the Faces Together

void connect_faces() 
{
	this->front_face.set_adjacent_top(&(this->top_face));
	this->front_face.set_adjacent_bottom(&(this->bottom_face));
	this->front_face.set_adjacent_left(&(this->left_face));
	this->front_face.set_adjacent_right(&(this->right_face));

	this->back_face.set_adjacent_top(&(this->top_face));
	this->back_face.set_adjacent_bottom(&(this->bottom_face));
	this->back_face.set_adjacent_left(&(this->left_face));
	this->back_face.set_adjacent_right(&(this->right_face));

	this->left_face.set_adjacent_top(&(this->top_face));
	this->left_face.set_adjacent_bottom(&(this->bottom_face));
	this->left_face.set_adjacent_left(&(this->left_face));
	this->left_face.set_adjacent_right(&(this->right_face));

	this->right_face.set_adjacent_top(&(this->top_face));
	this->right_face.set_adjacent_bottom(&(this->bottom_face));
	this->right_face.set_adjacent_left(&(this->left_face));
	this->right_face.set_adjacent_right(&(this->right_face));

	this->top_face.set_adjacent_top(&(this->top_face));
	this->top_face.set_adjacent_bottom(&(this->bottom_face));
	this->top_face.set_adjacent_left(&(this->left_face));
	this->top_face.set_adjacent_right(&(this->right_face));

	this->bottom_face.set_adjacent_top(&(this->top_face));
	this->bottom_face.set_adjacent_bottom(&(this->bottom_face));
	this->bottom_face.set_adjacent_left(&(this->left_face));
	this->bottom_face.set_adjacent_right(&(this->right_face));
}

////////////////////////////////////////////////////////////////////////////////

// Handle Printing
	
void Cube::print_cube() 
{
	Face blank_face;
	vector< vector<int> > net_rows;
	vector<int> temp_v1, temp_v2;

	temp_v1 = blank_face.get_row1();
	temp_v2 = this->top_face.get_row1();
	temp_v1.insert(temp_v1.end(), temp_v2.begin(), temp_v2.end());
	net_rows.push_back(temp_v1);
	temp_v1.clear();

	temp_v1 = blank_face.get_row2();
	temp_v2 = this->top_face.get_row2();
	temp_v1.insert(temp_v1.end(), temp_v2.begin(), temp_v2.end());
	net_rows.push_back(temp_v1);
	temp_v1.clear();

	temp_v1 = blank_face.get_row3();
	temp_v2 = this->top_face.get_row3();
	temp_v1.insert(temp_v1.end(), temp_v2.begin(), temp_v2.end());
	net_rows.push_back(temp_v1);
	temp_v1.clear();

	temp_v1 = this->left_face.get_row1();
	temp_v2 = this->front_face.get_row1();
	temp_v1.insert(temp_v1.end(), temp_v2.begin(), temp_v2.end());
	temp_v2 = this->right_face.get_row1();
	temp_v1.insert(temp_v1.end(), temp_v2.begin(), temp_v2.end());
	temp_v2 = this->back_face.get_row1();
	temp_v1.insert(temp_v1.end(), temp_v2.begin(), temp_v2.end());
	net_rows.push_back(temp_v1);
	temp_v1.clear();

	temp_v1 = this->left_face.get_row2();
	temp_v2 = this->front_face.get_row2();
	temp_v1.insert(temp_v1.end(), temp_v2.begin(), temp_v2.end());
	temp_v2 = this->right_face.get_row2();
	temp_v1.insert(temp_v1.end(), temp_v2.begin(), temp_v2.end());
	temp_v2 = this->back_face.get_row2();
	temp_v1.insert(temp_v1.end(), temp_v2.begin(), temp_v2.end());
	net_rows.push_back(temp_v1);
	temp_v1.clear();

	temp_v1 = this->left_face.get_row3();
	temp_v2 = this->front_face.get_row3();
	temp_v1.insert(temp_v1.end(), temp_v2.begin(), temp_v2.end());
	temp_v2 = this->right_face.get_row3();
	temp_v1.insert(temp_v1.end(), temp_v2.begin(), temp_v2.end());
	temp_v2 = this->back_face.get_row3();
	temp_v1.insert(temp_v1.end(), temp_v2.begin(), temp_v2.end());
	net_rows.push_back(temp_v1);
	temp_v1.clear();

	temp_v1 = blank_face.get_row1();
	temp_v2 = this->bottom_face.get_row1();
	temp_v1.insert(temp_v1.end(), temp_v2.begin(), temp_v2.end());
	net_rows.push_back(temp_v1);
	temp_v1.clear();

	temp_v1 = blank_face.get_row2();
	temp_v2 = this->bottom_face.get_row2();
	temp_v1.insert(temp_v1.end(), temp_v2.begin(), temp_v2.end());
	net_rows.push_back(temp_v1);
	temp_v1.clear();

	temp_v1 = blank_face.get_row3();
	temp_v2 = this->bottom_face.get_row3();
	temp_v1.insert(temp_v1.end(), temp_v2.begin(), temp_v2.end());
	net_rows.push_back(temp_v1);
	temp_v1.clear();

	vector< vector<int> >::iterator i;
	vector<int>::iterator j;
	
	for (i = net_rows.begin(); i != net_rows.end(); i++) 
	{
		for (j = (*i).begin(); j != (*i).end(); j++) 
		{
			print_square(*j);
		}
	
		cout << endl << endl;
	}

	cout << endl;
}

////////////////////////////////////////////////////////////////////////////////

