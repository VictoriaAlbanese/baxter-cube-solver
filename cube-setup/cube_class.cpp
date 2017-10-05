
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

	connect_faces();
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

	connect_faces();
} 

////////////////////////////////////////////////////////////////////////////////

// Connect the Faces Together

void Cube::connect_faces() 
{
	Face positioned_top;
	Face positioned_bottom;
	Face positioned_left;
	Face positioned_right;

	this->front_face.set_adjacent_top(&(this->top_face));
	this->front_face.set_adjacent_bottom(&(this->bottom_face));
	this->front_face.set_adjacent_left(&(this->left_face));
	this->front_face.set_adjacent_right(&(this->right_face));
	/*
	positioned_top = this->top_face;
	positioned_top.rotate_cw();
	positioned_top.rotate_cw();

	positioned_bottom = this->bottom_face;
	positioned_bottom.rotate_cw();
	positioned_bottom.rotate_cw();
	*/
	this->back_face.set_adjacent_top(&(this->top_face));
	this->back_face.set_adjacent_bottom(&(this->bottom_face));
	this->back_face.set_adjacent_left(&(this->right_face));
	this->back_face.set_adjacent_right(&(this->left_face));

	this->left_face.set_adjacent_top(&(this->top_face));
	this->left_face.set_adjacent_bottom(&(this->bottom_face));
	this->left_face.set_adjacent_left(&(this->back_face));
	this->left_face.set_adjacent_right(&(this->front_face));

	this->right_face.set_adjacent_top(&(this->top_face));
	this->right_face.set_adjacent_bottom(&(this->bottom_face));
	this->right_face.set_adjacent_left(&(this->front_face));
	this->right_face.set_adjacent_right(&(this->back_face));

	this->top_face.set_adjacent_top(&(this->back_face));
	this->top_face.set_adjacent_bottom(&(this->front_face));
	this->top_face.set_adjacent_left(&(this->left_face));
	this->top_face.set_adjacent_right(&(this->right_face));

	this->bottom_face.set_adjacent_top(&(this->front_face));
	this->bottom_face.set_adjacent_bottom(&(this->back_face));
	this->bottom_face.set_adjacent_left(&(this->left_face));
	this->bottom_face.set_adjacent_right(&(this->right_face));
}
	
////////////////////////////////////////////////////////////////////////////////

// Rotating Cube's Faces
	
void Cube::rotate_face_cw(int face_id) 
{
	Face face;
	switch (face_id) 
	{
		case FRONT:
			face = front_face;
			front_face.rotate_cw();
			break;			
		case BACK:
			face = back_face;
			back_face.rotate_cw();
			break;
		case TOP:
			face = top_face;
			top_face.rotate_cw();
			break;
		case BOTTOM:
			face = bottom_face;
			bottom_face.rotate_cw();
			break;
		case LEFT:
			face = left_face;
			left_face.rotate_cw();
			break;
		case RIGHT:
			face = right_face;
			right_face.rotate_cw();
			break;
	}
	
	vector<int> left_replacement_row = face.get_adjacent_bottom()->get_row1();
	vector<int> right_replacement_row = face.get_adjacent_top()->get_row3();

	rotate_cw_adjacent_top(face);
	rotate_cw_adjacent_bottom(face);
	rotate_adjacent_left(face, left_replacement_row);
	rotate_adjacent_right(face, right_replacement_row);
}

void Cube::rotate_face_ccw(int face_id) 
{
	Face face;
	switch (face_id) 
	{
		case FRONT:
			face = front_face;
			front_face.rotate_ccw();
			break;			
		case BACK:
			face = back_face;
			back_face.rotate_ccw();
			break;
		case TOP:
			face = top_face;
			top_face.rotate_ccw();
			break;
		case BOTTOM:
			face = bottom_face;
			bottom_face.rotate_ccw();
			break;
		case LEFT:
			face = left_face;
			left_face.rotate_ccw();
			break;
		case RIGHT:
			face = right_face;
			right_face.rotate_ccw();
			break;
	}
	
	vector<int> left_replacement_row = reverse_vector(face.get_adjacent_top()->get_row3());
	vector<int> right_replacement_row = reverse_vector(face.get_adjacent_bottom()->get_row1());

	rotate_ccw_adjacent_top(face);
	rotate_ccw_adjacent_bottom(face);
	rotate_adjacent_left(face, left_replacement_row);
	rotate_adjacent_right(face, right_replacement_row);
}

////////////////////////////////////////////////////////////////////////////////

// Rotation Helpers
		
void Cube::rotate_cw_adjacent_top(Face face) 
{
	vector<int> adjacent_left_column;
	Face adjacent_left_face = *face.get_adjacent_left();		
	
	adjacent_left_column.push_back(adjacent_left_face.get_row3()[2]);
	adjacent_left_column.push_back(adjacent_left_face.get_row2()[2]);
	adjacent_left_column.push_back(adjacent_left_face.get_row1()[2]);

	face.get_adjacent_top()->set_row3(adjacent_left_column);
}

void Cube::rotate_cw_adjacent_bottom(Face face)
{
	vector<int> adjacent_right_column;
	Face adjacent_right_face = *face.get_adjacent_right();		
	
	adjacent_right_column.push_back(adjacent_right_face.get_row1()[0]);
	adjacent_right_column.push_back(adjacent_right_face.get_row2()[0]);
	adjacent_right_column.push_back(adjacent_right_face.get_row3()[0]);

	face.get_adjacent_bottom()->set_row1(reverse_vector(adjacent_right_column));
}

void Cube::rotate_ccw_adjacent_top(Face face) 
{
	vector<int> adjacent_right_column;
	Face adjacent_right_face = *face.get_adjacent_right();		
	
	adjacent_right_column.push_back(adjacent_right_face.get_row1()[0]);
	adjacent_right_column.push_back(adjacent_right_face.get_row2()[0]);
	adjacent_right_column.push_back(adjacent_right_face.get_row3()[0]);

	face.get_adjacent_top()->set_row3(adjacent_right_column);
}

void Cube::rotate_ccw_adjacent_bottom(Face face)
{
	vector<int> adjacent_left_column;
	Face adjacent_left_face = *face.get_adjacent_left();		
	
	adjacent_left_column.push_back(adjacent_left_face.get_row1()[2]);
	adjacent_left_column.push_back(adjacent_left_face.get_row2()[2]);
	adjacent_left_column.push_back(adjacent_left_face.get_row3()[2]);

	face.get_adjacent_bottom()->set_row1(adjacent_left_column);
}

void Cube::rotate_adjacent_left(Face face, vector<int> replacement_row)
{
	Face adjacent_left_face = *face.get_adjacent_left();

	vector<int> new_adjacent_left_row1 = adjacent_left_face.get_row1();
	vector<int> new_adjacent_left_row2 = adjacent_left_face.get_row2();
	vector<int> new_adjacent_left_row3 = adjacent_left_face.get_row3();
	
	new_adjacent_left_row1[2] = replacement_row[0];
	new_adjacent_left_row2[2] = replacement_row[1];
	new_adjacent_left_row3[2] = replacement_row[2];

	face.get_adjacent_left()->set_row1(new_adjacent_left_row1);
	face.get_adjacent_left()->set_row2(new_adjacent_left_row2);
	face.get_adjacent_left()->set_row3(new_adjacent_left_row3);
}

void Cube::rotate_adjacent_right(Face face, vector<int> replacement_row)
{
	Face adjacent_right_face = *face.get_adjacent_right();

	vector<int> new_adjacent_right_row1 = adjacent_right_face.get_row1();
	vector<int> new_adjacent_right_row2 = adjacent_right_face.get_row2();
	vector<int> new_adjacent_right_row3 = adjacent_right_face.get_row3();
	
	new_adjacent_right_row1[0] = replacement_row[0];
	new_adjacent_right_row2[0] = replacement_row[1];
	new_adjacent_right_row3[0] = replacement_row[2];

	face.get_adjacent_right()->set_row1(new_adjacent_right_row1);
	face.get_adjacent_right()->set_row2(new_adjacent_right_row2);
	face.get_adjacent_right()->set_row3(new_adjacent_right_row3);
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

