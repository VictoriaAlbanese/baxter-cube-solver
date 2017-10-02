
////////////////////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// File name: face_class.cpp
// Purpose: Implement a class to represent a Rubik's cube's face
//
////////////////////////////////////////////////////////////////////////////////

#include "face_class.hpp"

using namespace std;

////////////////////////////////////////////////////////////////////////////////

// Constructors

Face::Face() 
{
	int color = -1;

	this->row1.push_back(color);
	this->row1.push_back(color);
	this->row1.push_back(color);

	this->row2 = this->row1;
	this->row3 = this->row1;
	
	this->face.push_back(this->row1);
	this->face.push_back(this->row2);
	this->face.push_back(this->row3);

	this->adjacent_top = this;
	this->adjacent_bottom = this;
	this->adjacent_left = this;
	this->adjacent_right = this;
}

Face::Face(vector< vector<int> > color_matrix) 
{
	if (color_matrix.size() != 3) 
	{
		cout << "Color matrix invalid size: must have three rows" << endl;
		exit(1);
	}

	this->adjacent_top = NULL;
	this->adjacent_bottom = NULL;
	this->adjacent_left = NULL;
	this->adjacent_right = NULL;

	this->row1 = color_matrix[0];
	this->row2 = color_matrix[1];
	this->row3 = color_matrix[2];
	
	this->face.push_back(this->row1);
	this->face.push_back(this->row2);
	this->face.push_back(this->row3);
}

bool Face::operator==(const Face &other) 
{
	bool is_equal = true;

	if (this->row1 != other.row1
		|| this->row2 != other.row2
		|| this->row3 != other.row3) 
	{
		is_equal = false;
	}
	
	return is_equal;
}

////////////////////////////////////////////////////////////////////////////////

// Rotating Functions

void Face::rotate_cw() 
{
	vector<int> temp_row1;
	vector<int> temp_row2;
	vector<int> temp_row3;

	temp_row1.push_back(this->row3[0]);
	temp_row1.push_back(this->row2[0]);
	temp_row1.push_back(this->row1[0]);

	temp_row2.push_back(this->row3[1]);
	temp_row2.push_back(this->row2[1]);
	temp_row2.push_back(this->row1[1]);

	temp_row3.push_back(this->row3[2]);
	temp_row3.push_back(this->row2[2]);
	temp_row3.push_back(this->row1[2]);

	vector< vector<int> > temp_face;
	temp_face.push_back(temp_row1);
	temp_face.push_back(temp_row2);
	temp_face.push_back(temp_row3);
	
	this->face = temp_face;
	this->row1 = temp_row1;
	this->row2 = temp_row2;
	this->row3 = temp_row3;
}

void Face::rotate_ccw() 
{
	vector<int> temp_row1;
	vector<int> temp_row2;
	vector<int> temp_row3;

	temp_row1.push_back(this->row1[2]);
	temp_row1.push_back(this->row2[2]);
	temp_row1.push_back(this->row3[2]);

	temp_row2.push_back(this->row1[1]);
	temp_row2.push_back(this->row2[1]);
	temp_row2.push_back(this->row3[1]);

	temp_row3.push_back(this->row1[0]);
	temp_row3.push_back(this->row2[0]);
	temp_row3.push_back(this->row3[0]);

	vector< vector<int> > temp_face;
	temp_face.push_back(temp_row1);
	temp_face.push_back(temp_row2);
	temp_face.push_back(temp_row3);
	
	this->face = temp_face;
	this->row1 = temp_row1;
	this->row2 = temp_row2;
	this->row3 = temp_row3;
}
		
////////////////////////////////////////////////////////////////////////////////

// Handle Printing

void Face::print_face() 
{
	cout << endl;

	vector< vector<int> >::iterator i;
	vector<int>::iterator j;
	
	for (i = this->face.begin(); i != this->face.end(); i++) {

		for (j = (*i).begin(); j != (*i).end(); j++) {
			print_square(*j);
		}

		cout << endl << endl;

	}

	cout << endl;
}
		
////////////////////////////////////////////////////////////////////////////////

