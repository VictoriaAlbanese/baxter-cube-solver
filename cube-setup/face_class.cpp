
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
	this->column1 = this->row1;
	this->column2 = this->row1;
	this->column3 = this->row1;
	
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

	if (color_matrix[0].size() != 3	|| color_matrix[1].size() != 3 || color_matrix[2].size() != 3) 
	{
		cout << "Color matrix invalid size: must have three columns" << endl;
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

	for (int i = 0; i < 3; i++) 
	{
		this->column1.push_back(color_matrix[i][0]);
	}

	for (int i = 0; i < 3; i++) 
	{
		this->column2.push_back(color_matrix[i][1]);
	}

	for (int i = 0; i < 3; i++) 
	{
		this->column3.push_back(color_matrix[i][2]);
	}

	
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
	vector<int> temp_column1 = reverse_vector(this->column1);
	vector<int> temp_column2 = reverse_vector(this->column2);
	vector<int> temp_column3 = reverse_vector(this->column3);

	this->column1 = this->row3;
	this->column2 = this->row2;
	this->column3 = this->row1;

	this->row1 = temp_column1;
	this->row2 = temp_column2;
	this->row3 = temp_column3;

	vector< vector<int> > temp_face;
	temp_face.push_back(this->row1);
	temp_face.push_back(this->row2);
	temp_face.push_back(this->row3);
	
	this->face = temp_face;
}

void Face::rotate_ccw() 
{
	vector<int> temp_row1 = reverse_vector(this->row1);
	vector<int> temp_row2 = reverse_vector(this->row2);
	vector<int> temp_row3 = reverse_vector(this->row3);

	this->row1 = this->column3;
	this->row2 = this->column2;
	this->row3 = this->column1;

	this->column1 = temp_row1;
	this->column2 = temp_row2;
	this->column3 = temp_row3;

	vector< vector<int> > temp_face;
	temp_face.push_back(row1);
	temp_face.push_back(row2);
	temp_face.push_back(row3);

	this->face = temp_face;
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

