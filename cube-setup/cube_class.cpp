
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
	this->front_face.set_adjacent_top(&(this->top_face));
	this->front_face.set_adjacent_bottom(&(this->bottom_face));
	this->front_face.set_adjacent_left(&(this->left_face));
	this->front_face.set_adjacent_right(&(this->right_face));
	
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

// Rotating Front Face
	
void Cube::rotate_front_face_cw() 
{
	// rotate the face itself
	front_face.rotate_cw();
	
	// save some cube state info
	vector<int> left_replacement_row = front_face.get_adjacent_bottom()->get_row1();
	vector<int> right_replacement_row = front_face.get_adjacent_top()->get_row3();

	// fix the adjacent top face
	vector<int> adjacent_left_column;
	Face adjacent_left_face = *front_face.get_adjacent_left();		
	adjacent_left_column.push_back(adjacent_left_face.get_row3()[2]);
	adjacent_left_column.push_back(adjacent_left_face.get_row2()[2]);
	adjacent_left_column.push_back(adjacent_left_face.get_row1()[2]);
	front_face.get_adjacent_top()->set_row3(adjacent_left_column);

	// fix the adjacent bottom face
	vector<int> adjacent_right_column;
	Face adjacent_right_face = *front_face.get_adjacent_right();		
	adjacent_right_column.push_back(adjacent_right_face.get_row1()[0]);
	adjacent_right_column.push_back(adjacent_right_face.get_row2()[0]);
	adjacent_right_column.push_back(adjacent_right_face.get_row3()[0]);
	front_face.get_adjacent_bottom()->set_row1(reverse_vector(adjacent_right_column));

	// fix the adjacent left face
	vector<int> new_adjacent_left_row1 = adjacent_left_face.get_row1();
	vector<int> new_adjacent_left_row2 = adjacent_left_face.get_row2();
	vector<int> new_adjacent_left_row3 = adjacent_left_face.get_row3();
	new_adjacent_left_row1[2] = left_replacement_row[0];
	new_adjacent_left_row2[2] = left_replacement_row[1];
	new_adjacent_left_row3[2] = left_replacement_row[2];
	front_face.get_adjacent_left()->set_row1(new_adjacent_left_row1);
	front_face.get_adjacent_left()->set_row2(new_adjacent_left_row2);
	front_face.get_adjacent_left()->set_row3(new_adjacent_left_row3);

	// fix the adjacent right face
	vector<int> new_adjacent_right_row1 = adjacent_right_face.get_row1();
	vector<int> new_adjacent_right_row2 = adjacent_right_face.get_row2();
	vector<int> new_adjacent_right_row3 = adjacent_right_face.get_row3();
	new_adjacent_right_row1[0] = right_replacement_row[0];
	new_adjacent_right_row2[0] = right_replacement_row[1];
	new_adjacent_right_row3[0] = right_replacement_row[2];
	front_face.get_adjacent_right()->set_row1(new_adjacent_right_row1);
	front_face.get_adjacent_right()->set_row2(new_adjacent_right_row2);
	front_face.get_adjacent_right()->set_row3(new_adjacent_right_row3);
}

void Cube::rotate_front_face_ccw() 
{
	// rotate the face itself
	front_face.rotate_ccw();	
	
	// save some cube state info
	vector<int> left_replacement_row = reverse_vector(front_face.get_adjacent_top()->get_row3());
	vector<int> right_replacement_row = reverse_vector(front_face.get_adjacent_bottom()->get_row1());

	// fix the adjacent top face
	vector<int> adjacent_right_column;
	Face adjacent_right_face = *front_face.get_adjacent_right();		
	adjacent_right_column.push_back(adjacent_right_face.get_row1()[0]);
	adjacent_right_column.push_back(adjacent_right_face.get_row2()[0]);
	adjacent_right_column.push_back(adjacent_right_face.get_row3()[0]);
	front_face.get_adjacent_top()->set_row3(adjacent_right_column);

	// fix the adjacent bottom face
	vector<int> adjacent_left_column;
	Face adjacent_left_face = *front_face.get_adjacent_left();		
	adjacent_left_column.push_back(adjacent_left_face.get_row1()[2]);
	adjacent_left_column.push_back(adjacent_left_face.get_row2()[2]);
	adjacent_left_column.push_back(adjacent_left_face.get_row3()[2]);
	front_face.get_adjacent_bottom()->set_row1(adjacent_left_column);

	// fix the adjacent left face
	vector<int> new_adjacent_left_row1 = adjacent_left_face.get_row1();
	vector<int> new_adjacent_left_row2 = adjacent_left_face.get_row2();
	vector<int> new_adjacent_left_row3 = adjacent_left_face.get_row3();
	new_adjacent_left_row1[2] = left_replacement_row[0];
	new_adjacent_left_row2[2] = left_replacement_row[1];
	new_adjacent_left_row3[2] = left_replacement_row[2];
	front_face.get_adjacent_left()->set_row1(new_adjacent_left_row1);
	front_face.get_adjacent_left()->set_row2(new_adjacent_left_row2);
	front_face.get_adjacent_left()->set_row3(new_adjacent_left_row3);

	// fix the adjacent right face
	vector<int> new_adjacent_right_row1 = adjacent_right_face.get_row1();
	vector<int> new_adjacent_right_row2 = adjacent_right_face.get_row2();
	vector<int> new_adjacent_right_row3 = adjacent_right_face.get_row3();
	new_adjacent_right_row1[0] = right_replacement_row[0];
	new_adjacent_right_row2[0] = right_replacement_row[1];
	new_adjacent_right_row3[0] = right_replacement_row[2];
	front_face.get_adjacent_right()->set_row1(new_adjacent_right_row1);
	front_face.get_adjacent_right()->set_row2(new_adjacent_right_row2);
	front_face.get_adjacent_right()->set_row3(new_adjacent_right_row3);
}

////////////////////////////////////////////////////////////////////////////////

// Rotating Back Face
	
void Cube::rotate_back_face_cw() 
{
	// rotate the face itself
	back_face.rotate_cw();
	
	// save some cube state info
	vector<int> left_replacement_row = back_face.get_adjacent_bottom()->get_row3();
	vector<int> right_replacement_row = back_face.get_adjacent_top()->get_row1();

	// fix the adjacent top face
	vector<int> adjacent_left_column;
	Face adjacent_left_face = *back_face.get_adjacent_left();		
	adjacent_left_column.push_back(adjacent_left_face.get_row1()[2]);
	adjacent_left_column.push_back(adjacent_left_face.get_row2()[2]);
	adjacent_left_column.push_back(adjacent_left_face.get_row3()[2]);
	back_face.get_adjacent_top()->set_row1(adjacent_left_column);

	// fix the adjacent bottom face
	vector<int> adjacent_right_column;
	Face adjacent_right_face = *back_face.get_adjacent_right();		
	adjacent_right_column.push_back(adjacent_right_face.get_row3()[0]);
	adjacent_right_column.push_back(adjacent_right_face.get_row2()[0]);
	adjacent_right_column.push_back(adjacent_right_face.get_row1()[0]);
	back_face.get_adjacent_bottom()->set_row3(reverse_vector(adjacent_right_column));

	// fix the adjacent left face
	vector<int> new_adjacent_left_row1 = adjacent_left_face.get_row1();
	vector<int> new_adjacent_left_row2 = adjacent_left_face.get_row2();
	vector<int> new_adjacent_left_row3 = adjacent_left_face.get_row3();
	new_adjacent_left_row3[2] = left_replacement_row[0];
	new_adjacent_left_row2[2] = left_replacement_row[1];
	new_adjacent_left_row1[2] = left_replacement_row[2];
	back_face.get_adjacent_left()->set_row1(new_adjacent_left_row1);
	back_face.get_adjacent_left()->set_row2(new_adjacent_left_row2);
	back_face.get_adjacent_left()->set_row3(new_adjacent_left_row3);

	// fix the adjacent right face
	vector<int> new_adjacent_right_row1 = adjacent_right_face.get_row1();
	vector<int> new_adjacent_right_row2 = adjacent_right_face.get_row2();
	vector<int> new_adjacent_right_row3 = adjacent_right_face.get_row3();
	new_adjacent_right_row3[0] = right_replacement_row[0];
	new_adjacent_right_row2[0] = right_replacement_row[1];
	new_adjacent_right_row1[0] = right_replacement_row[2];
	back_face.get_adjacent_right()->set_row1(new_adjacent_right_row1);
	back_face.get_adjacent_right()->set_row2(new_adjacent_right_row2);
	back_face.get_adjacent_right()->set_row3(new_adjacent_right_row3);
}

void Cube::rotate_back_face_ccw() 
{
	// rotate the face itself
	back_face.rotate_ccw();	
	
	// save some cube state info
	vector<int> left_replacement_row = reverse_vector(back_face.get_adjacent_top()->get_row1());
	vector<int> right_replacement_row = reverse_vector(back_face.get_adjacent_bottom()->get_row3());

	// fix the adjacent top face
	vector<int> adjacent_right_column;
	Face adjacent_right_face = *back_face.get_adjacent_right();		
	adjacent_right_column.push_back(adjacent_right_face.get_row3()[0]);
	adjacent_right_column.push_back(adjacent_right_face.get_row2()[0]);
	adjacent_right_column.push_back(adjacent_right_face.get_row1()[0]);
	back_face.get_adjacent_top()->set_row1(adjacent_right_column);

	// fix the adjacent bottom face
	vector<int> adjacent_left_column;
	Face adjacent_left_face = *back_face.get_adjacent_left();		
	adjacent_left_column.push_back(adjacent_left_face.get_row3()[2]);
	adjacent_left_column.push_back(adjacent_left_face.get_row2()[2]);
	adjacent_left_column.push_back(adjacent_left_face.get_row1()[2]);
	back_face.get_adjacent_bottom()->set_row3(adjacent_left_column);

	// fix the adjacent left face
	vector<int> new_adjacent_left_row1 = adjacent_left_face.get_row1();
	vector<int> new_adjacent_left_row2 = adjacent_left_face.get_row2();
	vector<int> new_adjacent_left_row3 = adjacent_left_face.get_row3();
	new_adjacent_left_row3[2] = left_replacement_row[0];
	new_adjacent_left_row2[2] = left_replacement_row[1];
	new_adjacent_left_row1[2] = left_replacement_row[2];
	back_face.get_adjacent_left()->set_row1(new_adjacent_left_row1);
	back_face.get_adjacent_left()->set_row2(new_adjacent_left_row2);
	back_face.get_adjacent_left()->set_row3(new_adjacent_left_row3);

	// fix the adjacent right face
	vector<int> new_adjacent_right_row1 = adjacent_right_face.get_row1();
	vector<int> new_adjacent_right_row2 = adjacent_right_face.get_row2();
	vector<int> new_adjacent_right_row3 = adjacent_right_face.get_row3();
	new_adjacent_right_row3[0] = right_replacement_row[0];
	new_adjacent_right_row2[0] = right_replacement_row[1];
	new_adjacent_right_row1[0] = right_replacement_row[2];
	back_face.get_adjacent_right()->set_row1(new_adjacent_right_row1);
	back_face.get_adjacent_right()->set_row2(new_adjacent_right_row2);
	back_face.get_adjacent_right()->set_row3(new_adjacent_right_row3);
}

////////////////////////////////////////////////////////////////////////////////

// Rotating Top Face
	
void Cube::rotate_top_face_cw() 
{
	// rotate the face itself
	top_face.rotate_cw();
	
	// save some cube state info
	vector<int> left_replacement_row = top_face.get_adjacent_bottom()->get_row1();
	vector<int> right_replacement_row = top_face.get_adjacent_top()->get_row1();

	// fix the adjacent top face
	top_face.get_adjacent_top()->set_row1(top_face.get_adjacent_left()->get_row1());

	// fix the adjacent bottom face
	top_face.get_adjacent_bottom()->set_row1(top_face.get_adjacent_right()->get_row1());

	// fix the adjacent left face
	top_face.get_adjacent_left()->set_row1(left_replacement_row);

	// fix the adjacent right face
	top_face.get_adjacent_right()->set_row1(right_replacement_row);
}

void Cube::rotate_top_face_ccw() 
{
	// rotate the face itself
	top_face.rotate_ccw();	
	
	// save some cube state info
	vector<int> left_replacement_row = reverse_vector(top_face.get_adjacent_top()->get_row1());
	vector<int> right_replacement_row = reverse_vector(top_face.get_adjacent_bottom()->get_row1());

	// fix the adjacent top face
	top_face.get_adjacent_top()->set_row1(top_face.get_adjacent_right()->get_row1());		

	// fix the adjacent bottom face
	top_face.get_adjacent_bottom()->set_row1(top_face.get_adjacent_left()->get_row1());		

	// fix the adjacent left face
	top_face.get_adjacent_left()->set_row1(reverse_vector(left_replacement_row));

	// fix the adjacent right face
	top_face.get_adjacent_right()->set_row1(reverse_vector(right_replacement_row));
}

////////////////////////////////////////////////////////////////////////////////

// Rotating Bottom Face
	
void Cube::rotate_bottom_face_cw() 
{
	// rotate the face itself
	bottom_face.rotate_cw();
	
	// save some cube state info
	vector<int> left_replacement_row = bottom_face.get_adjacent_bottom()->get_row3();
	vector<int> right_replacement_row = bottom_face.get_adjacent_top()->get_row3();

	// fix the adjacent top face
	bottom_face.get_adjacent_top()->set_row3(bottom_face.get_adjacent_left()->get_row3());

	// fix the adjacent bottom face
	bottom_face.get_adjacent_bottom()->set_row3(reverse_vector(reverse_vector(bottom_face.get_adjacent_right()->get_row3())));

	// fix the adjacent left face
	bottom_face.get_adjacent_left()->set_row3(left_replacement_row);

	// fix the adjacent right face
	bottom_face.get_adjacent_right()->set_row3(right_replacement_row);
}

void Cube::rotate_bottom_face_ccw() 
{
	// rotate the face itself
	bottom_face.rotate_ccw();	
	
	// save some cube state info
	vector<int> left_replacement_row = reverse_vector(bottom_face.get_adjacent_top()->get_row3());
	vector<int> right_replacement_row = reverse_vector(bottom_face.get_adjacent_bottom()->get_row3());

	// fix the adjacent top face
	bottom_face.get_adjacent_top()->set_row3(top_face.get_adjacent_right()->get_row3());		

	// fix the adjacent bottom face
	bottom_face.get_adjacent_bottom()->set_row3(top_face.get_adjacent_left()->get_row3());		

	// fix the adjacent left face
	bottom_face.get_adjacent_left()->set_row3(reverse_vector(left_replacement_row));

	// fix the adjacent right face
	bottom_face.get_adjacent_right()->set_row3(right_replacement_row);
}

////////////////////////////////////////////////////////////////////////////////

// Rotating Left Face
	
void Cube::rotate_left_face_cw() 
{
	// rotate the face itself
	left_face.rotate_cw();
	
	// save some cube state info
	vector<int> left_replacement_row;
	left_replacement_row.push_back(right_face.get_adjacent_bottom()->get_row3()[0]);
	left_replacement_row.push_back(right_face.get_adjacent_bottom()->get_row2()[0]);
	left_replacement_row.push_back(right_face.get_adjacent_bottom()->get_row1()[0]);
	vector<int> right_replacement_row;
	right_replacement_row.push_back(right_face.get_adjacent_top()->get_row1()[0]);
	right_replacement_row.push_back(right_face.get_adjacent_top()->get_row2()[0]);
	right_replacement_row.push_back(right_face.get_adjacent_top()->get_row3()[0]);

	// fix the adjacent top face
	vector<int> adjacent_left_column;
	Face adjacent_left_face = *left_face.get_adjacent_left();		
	Face adjacent_top_face = *left_face.get_adjacent_top();		
	adjacent_left_column.push_back(adjacent_left_face.get_row3()[2]);
	adjacent_left_column.push_back(adjacent_left_face.get_row2()[2]);
	adjacent_left_column.push_back(adjacent_left_face.get_row1()[2]);
	vector<int> new_adjacent_top_row1 = adjacent_top_face.get_row1();
	vector<int> new_adjacent_top_row2 = adjacent_top_face.get_row2();
	vector<int> new_adjacent_top_row3 = adjacent_top_face.get_row3();
	new_adjacent_top_row1[0] = adjacent_left_column[0];
	new_adjacent_top_row2[0] = adjacent_left_column[1];
	new_adjacent_top_row3[0] = adjacent_left_column[2];
	left_face.get_adjacent_top()->set_row1(new_adjacent_top_row1);
	left_face.get_adjacent_top()->set_row2(new_adjacent_top_row2);
	left_face.get_adjacent_top()->set_row3(new_adjacent_top_row3);

	// fix the adjacent bottom face
	vector<int> adjacent_right_column;
	Face adjacent_right_face = *left_face.get_adjacent_right();		
	Face adjacent_bottom_face = *left_face.get_adjacent_bottom();		
	adjacent_right_column.push_back(adjacent_right_face.get_row1()[0]);
	adjacent_right_column.push_back(adjacent_right_face.get_row2()[0]);
	adjacent_right_column.push_back(adjacent_right_face.get_row3()[0]);
	vector<int> new_adjacent_bottom_row1 = adjacent_bottom_face.get_row1();
	vector<int> new_adjacent_bottom_row2 = adjacent_bottom_face.get_row2();
	vector<int> new_adjacent_bottom_row3 = adjacent_bottom_face.get_row3();
	new_adjacent_bottom_row1[0] = adjacent_right_column[0];
	new_adjacent_bottom_row2[0] = adjacent_right_column[1];
	new_adjacent_bottom_row3[0] = adjacent_right_column[2];
	left_face.get_adjacent_bottom()->set_row1(new_adjacent_bottom_row1);
	left_face.get_adjacent_bottom()->set_row2(new_adjacent_bottom_row2);
	left_face.get_adjacent_bottom()->set_row3(new_adjacent_bottom_row3);

	// fix the adjacent left face
	vector<int> new_adjacent_left_row1 = adjacent_left_face.get_row1();
	vector<int> new_adjacent_left_row2 = adjacent_left_face.get_row2();
	vector<int> new_adjacent_left_row3 = adjacent_left_face.get_row3();
	new_adjacent_left_row1[2] = left_replacement_row[0];
	new_adjacent_left_row2[2] = left_replacement_row[1];
	new_adjacent_left_row3[2] = left_replacement_row[2];
	left_face.get_adjacent_left()->set_row1(new_adjacent_left_row1);
	left_face.get_adjacent_left()->set_row2(new_adjacent_left_row2);
	left_face.get_adjacent_left()->set_row3(new_adjacent_left_row3);

	// fix the adjacent right face
	vector<int> new_adjacent_right_row1 = adjacent_right_face.get_row1();
	vector<int> new_adjacent_right_row2 = adjacent_right_face.get_row2();
	vector<int> new_adjacent_right_row3 = adjacent_right_face.get_row3();
	new_adjacent_right_row1[0] = right_replacement_row[0];
	new_adjacent_right_row2[0] = right_replacement_row[1];
	new_adjacent_right_row3[0] = right_replacement_row[2];
	left_face.get_adjacent_right()->set_row1(new_adjacent_right_row1);
	left_face.get_adjacent_right()->set_row2(new_adjacent_right_row2);
	left_face.get_adjacent_right()->set_row3(new_adjacent_right_row3);
}

void Cube::rotate_left_face_ccw() 
{
	// rotate the face itself
	left_face.rotate_ccw();	
	
	// save some cube state info
	vector<int> left_replacement_row;
	left_replacement_row.push_back(right_face.get_adjacent_top()->get_row3()[0]);
	left_replacement_row.push_back(right_face.get_adjacent_top()->get_row2()[0]);
	left_replacement_row.push_back(right_face.get_adjacent_top()->get_row1()[0]);
	vector<int> right_replacement_row;
	right_replacement_row.push_back(right_face.get_adjacent_bottom()->get_row1()[0]);
	right_replacement_row.push_back(right_face.get_adjacent_bottom()->get_row2()[0]);
	right_replacement_row.push_back(right_face.get_adjacent_bottom()->get_row3()[0]);

	// fix the adjacent top face
	vector<int> adjacent_right_column;
	Face adjacent_right_face = *left_face.get_adjacent_right();		
	Face adjacent_top_face = *left_face.get_adjacent_top();		
	adjacent_right_column.push_back(adjacent_right_face.get_row1()[0]);
	adjacent_right_column.push_back(adjacent_right_face.get_row2()[0]);
	adjacent_right_column.push_back(adjacent_right_face.get_row3()[0]);
	vector<int> new_adjacent_top_row1 = adjacent_top_face.get_row1();
	vector<int> new_adjacent_top_row2 = adjacent_top_face.get_row2();
	vector<int> new_adjacent_top_row3 = adjacent_top_face.get_row3();
	new_adjacent_top_row3[0] = adjacent_right_column[0];
	new_adjacent_top_row2[0] = adjacent_right_column[1];
	new_adjacent_top_row1[0] = adjacent_right_column[2];
	left_face.get_adjacent_top()->set_row1(new_adjacent_top_row1);
	left_face.get_adjacent_top()->set_row2(new_adjacent_top_row2);
	left_face.get_adjacent_top()->set_row3(new_adjacent_top_row3);

	// fix the adjacent bottom face
	vector<int> adjacent_left_column;
	Face adjacent_left_face = *left_face.get_adjacent_left();		
	Face adjacent_bottom_face = *left_face.get_adjacent_bottom();		
	adjacent_left_column.push_back(adjacent_left_face.get_row1()[2]);
	adjacent_left_column.push_back(adjacent_left_face.get_row2()[2]);
	adjacent_left_column.push_back(adjacent_left_face.get_row3()[2]);
	vector<int> new_adjacent_bottom_row1 = adjacent_bottom_face.get_row1();
	vector<int> new_adjacent_bottom_row2 = adjacent_bottom_face.get_row2();
	vector<int> new_adjacent_bottom_row3 = adjacent_bottom_face.get_row3();
	new_adjacent_bottom_row3[0] = adjacent_left_column[0];
	new_adjacent_bottom_row2[0] = adjacent_left_column[1];
	new_adjacent_bottom_row1[0] = adjacent_left_column[2];
	left_face.get_adjacent_bottom()->set_row1(new_adjacent_bottom_row1);
	left_face.get_adjacent_bottom()->set_row2(new_adjacent_bottom_row2);
	left_face.get_adjacent_bottom()->set_row3(new_adjacent_bottom_row3);

	// fix the adjacent left face
	vector<int> new_adjacent_left_row1 = adjacent_left_face.get_row1();
	vector<int> new_adjacent_left_row2 = adjacent_left_face.get_row2();
	vector<int> new_adjacent_left_row3 = adjacent_left_face.get_row3();
	new_adjacent_left_row1[2] = left_replacement_row[0];
	new_adjacent_left_row2[2] = left_replacement_row[1];
	new_adjacent_left_row3[2] = left_replacement_row[2];
	left_face.get_adjacent_left()->set_row1(new_adjacent_left_row1);
	left_face.get_adjacent_left()->set_row2(new_adjacent_left_row2);
	left_face.get_adjacent_left()->set_row3(new_adjacent_left_row3);

	// fix the adjacent right face
	vector<int> new_adjacent_right_row1 = adjacent_right_face.get_row1();
	vector<int> new_adjacent_right_row2 = adjacent_right_face.get_row2();
	vector<int> new_adjacent_right_row3 = adjacent_right_face.get_row3();
	new_adjacent_right_row1[0] = right_replacement_row[0];
	new_adjacent_right_row2[0] = right_replacement_row[1];
	new_adjacent_right_row3[0] = right_replacement_row[2];
	left_face.get_adjacent_right()->set_row1(new_adjacent_right_row1);
	left_face.get_adjacent_right()->set_row2(new_adjacent_right_row2);
	left_face.get_adjacent_right()->set_row3(new_adjacent_right_row3);
}

////////////////////////////////////////////////////////////////////////////////

// Rotating Right Face
	
void Cube::rotate_right_face_cw() 
{
	// rotate the face itself
	right_face.rotate_cw();
	
	// save some cube state info
	vector<int> left_replacement_row;
	left_replacement_row.push_back(right_face.get_adjacent_bottom()->get_row1()[2]);
	left_replacement_row.push_back(right_face.get_adjacent_bottom()->get_row2()[2]);
	left_replacement_row.push_back(right_face.get_adjacent_bottom()->get_row3()[2]);
	vector<int> right_replacement_row;
	right_replacement_row.push_back(right_face.get_adjacent_top()->get_row3()[2]);
	right_replacement_row.push_back(right_face.get_adjacent_top()->get_row2()[2]);
	right_replacement_row.push_back(right_face.get_adjacent_top()->get_row1()[2]);

	// fix the adjacent top face
	vector<int> adjacent_left_column;
	Face adjacent_left_face = *right_face.get_adjacent_left();		
	Face adjacent_top_face = *right_face.get_adjacent_top();		
	adjacent_left_column.push_back(adjacent_left_face.get_row3()[2]);
	adjacent_left_column.push_back(adjacent_left_face.get_row2()[2]);
	adjacent_left_column.push_back(adjacent_left_face.get_row1()[2]);
	vector<int> new_adjacent_top_row1 = adjacent_top_face.get_row1();
	vector<int> new_adjacent_top_row2 = adjacent_top_face.get_row2();
	vector<int> new_adjacent_top_row3 = adjacent_top_face.get_row3();
	new_adjacent_top_row3[2] = adjacent_left_column[0];
	new_adjacent_top_row2[2] = adjacent_left_column[1];
	new_adjacent_top_row1[2] = adjacent_left_column[2];
	right_face.get_adjacent_top()->set_row1(new_adjacent_top_row1);
	right_face.get_adjacent_top()->set_row2(new_adjacent_top_row2);
	right_face.get_adjacent_top()->set_row3(new_adjacent_top_row3);

	// fix the adjacent bottom face
	vector<int> adjacent_right_column;
	Face adjacent_right_face = *right_face.get_adjacent_right();		
	Face adjacent_bottom_face = *right_face.get_adjacent_bottom();		
	adjacent_right_column.push_back(adjacent_right_face.get_row1()[0]);
	adjacent_right_column.push_back(adjacent_right_face.get_row2()[0]);
	adjacent_right_column.push_back(adjacent_right_face.get_row3()[0]);
	vector<int> new_adjacent_bottom_row1 = adjacent_bottom_face.get_row1();
	vector<int> new_adjacent_bottom_row2 = adjacent_bottom_face.get_row2();
	vector<int> new_adjacent_bottom_row3 = adjacent_bottom_face.get_row3();
	new_adjacent_bottom_row3[2] = adjacent_right_column[0];
	new_adjacent_bottom_row2[2] = adjacent_right_column[1];
	new_adjacent_bottom_row1[2] = adjacent_right_column[2];
	right_face.get_adjacent_bottom()->set_row1(new_adjacent_bottom_row1);
	right_face.get_adjacent_bottom()->set_row2(new_adjacent_bottom_row2);
	right_face.get_adjacent_bottom()->set_row3(new_adjacent_bottom_row3);

	// fix the adjacent left face
	vector<int> new_adjacent_left_row1 = adjacent_left_face.get_row1();
	vector<int> new_adjacent_left_row2 = adjacent_left_face.get_row2();
	vector<int> new_adjacent_left_row3 = adjacent_left_face.get_row3();
	new_adjacent_left_row1[2] = left_replacement_row[0];
	new_adjacent_left_row2[2] = left_replacement_row[1];
	new_adjacent_left_row3[2] = left_replacement_row[2];
	right_face.get_adjacent_left()->set_row1(new_adjacent_left_row1);
	right_face.get_adjacent_left()->set_row2(new_adjacent_left_row2);
	right_face.get_adjacent_left()->set_row3(new_adjacent_left_row3);

	// fix the adjacent right face
	vector<int> new_adjacent_right_row1 = adjacent_right_face.get_row1();
	vector<int> new_adjacent_right_row2 = adjacent_right_face.get_row2();
	vector<int> new_adjacent_right_row3 = adjacent_right_face.get_row3();
	new_adjacent_right_row1[0] = right_replacement_row[0];
	new_adjacent_right_row2[0] = right_replacement_row[1];
	new_adjacent_right_row3[0] = right_replacement_row[2];
	right_face.get_adjacent_right()->set_row1(new_adjacent_right_row1);
	right_face.get_adjacent_right()->set_row2(new_adjacent_right_row2);
	right_face.get_adjacent_right()->set_row3(new_adjacent_right_row3);
}

void Cube::rotate_right_face_ccw() 
{
	// rotate the face itself
	right_face.rotate_ccw();	
	
	// save some cube state info
	vector<int> left_replacement_row;
	left_replacement_row.push_back(right_face.get_adjacent_top()->get_row1()[2]);
	left_replacement_row.push_back(right_face.get_adjacent_top()->get_row2()[2]);
	left_replacement_row.push_back(right_face.get_adjacent_top()->get_row3()[2]);
	vector<int> right_replacement_row;
	right_replacement_row.push_back(right_face.get_adjacent_bottom()->get_row3()[2]);
	right_replacement_row.push_back(right_face.get_adjacent_bottom()->get_row2()[2]);
	right_replacement_row.push_back(right_face.get_adjacent_bottom()->get_row1()[2]);

	// fix the adjacent top face
	vector<int> adjacent_right_column;
	Face adjacent_right_face = *right_face.get_adjacent_right();		
	Face adjacent_top_face = *right_face.get_adjacent_top();		
	adjacent_right_column.push_back(adjacent_right_face.get_row1()[0]);
	adjacent_right_column.push_back(adjacent_right_face.get_row2()[0]);
	adjacent_right_column.push_back(adjacent_right_face.get_row3()[0]);
	vector<int> new_adjacent_top_row1 = adjacent_top_face.get_row1();
	vector<int> new_adjacent_top_row2 = adjacent_top_face.get_row2();
	vector<int> new_adjacent_top_row3 = adjacent_top_face.get_row3();
	new_adjacent_top_row1[2] = adjacent_right_column[0];
	new_adjacent_top_row2[2] = adjacent_right_column[1];
	new_adjacent_top_row3[2] = adjacent_right_column[2];
	right_face.get_adjacent_top()->set_row1(new_adjacent_top_row1);
	right_face.get_adjacent_top()->set_row2(new_adjacent_top_row2);
	right_face.get_adjacent_top()->set_row3(new_adjacent_top_row3);

	// fix the adjacent bottom face
	vector<int> adjacent_left_column;
	Face adjacent_left_face = *right_face.get_adjacent_left();		
	Face adjacent_bottom_face = *right_face.get_adjacent_bottom();		
	adjacent_left_column.push_back(adjacent_left_face.get_row1()[2]);
	adjacent_left_column.push_back(adjacent_left_face.get_row2()[2]);
	adjacent_left_column.push_back(adjacent_left_face.get_row3()[2]);
	vector<int> new_adjacent_bottom_row1 = adjacent_bottom_face.get_row1();
	vector<int> new_adjacent_bottom_row2 = adjacent_bottom_face.get_row2();
	vector<int> new_adjacent_bottom_row3 = adjacent_bottom_face.get_row3();
	new_adjacent_bottom_row1[2] = adjacent_left_column[0];
	new_adjacent_bottom_row2[2] = adjacent_left_column[1];
	new_adjacent_bottom_row3[2] = adjacent_left_column[2];
	right_face.get_adjacent_bottom()->set_row1(new_adjacent_bottom_row1);
	right_face.get_adjacent_bottom()->set_row2(new_adjacent_bottom_row2);
	right_face.get_adjacent_bottom()->set_row3(new_adjacent_bottom_row3);

	// fix the adjacent left face
	vector<int> new_adjacent_left_row1 = adjacent_left_face.get_row1();
	vector<int> new_adjacent_left_row2 = adjacent_left_face.get_row2();
	vector<int> new_adjacent_left_row3 = adjacent_left_face.get_row3();
	new_adjacent_left_row1[2] = left_replacement_row[0];
	new_adjacent_left_row2[2] = left_replacement_row[1];
	new_adjacent_left_row3[2] = left_replacement_row[2];
	right_face.get_adjacent_left()->set_row1(new_adjacent_left_row1);
	right_face.get_adjacent_left()->set_row2(new_adjacent_left_row2);
	right_face.get_adjacent_left()->set_row3(new_adjacent_left_row3);

	// fix the adjacent right face
	vector<int> new_adjacent_right_row1 = adjacent_right_face.get_row1();
	vector<int> new_adjacent_right_row2 = adjacent_right_face.get_row2();
	vector<int> new_adjacent_right_row3 = adjacent_right_face.get_row3();
	new_adjacent_right_row1[0] = right_replacement_row[0];
	new_adjacent_right_row2[0] = right_replacement_row[1];
	new_adjacent_right_row3[0] = right_replacement_row[2];
	right_face.get_adjacent_right()->set_row1(new_adjacent_right_row1);
	right_face.get_adjacent_right()->set_row2(new_adjacent_right_row2);
	right_face.get_adjacent_right()->set_row3(new_adjacent_right_row3);
}

////////////////////////////////////////////////////////////////////////////////

// Helper Functions 
/*
vector<int> get_column(Face face, int column) 
{
	vector<int> column;

	column.push_back(face.get_row1()[column]);
	column.push_back(face.get_row2()[column]);
	column.push_back(face.get_row3()[column]);

	return column;
}
*/
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

