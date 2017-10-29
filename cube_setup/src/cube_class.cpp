
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
	if (faces.size() != 6) 
	{
		throw invalid_argument("Invalid number of faces: expected 6");
	}

	this->front_face = faces[0];
	this->back_face = faces[1];
	this->top_face = faces[2];
	this->bottom_face = faces[3];
	this->left_face = faces[4];
	this->right_face = faces[5];

	connect_faces();
} 

bool Cube::operator==(const Cube &other) const 
{
	bool is_same = true;

	if (this->front_face != other.front_face
		|| this->back_face != other.back_face
		|| this->top_face != other.top_face
		|| this->bottom_face != other.bottom_face
		|| this->left_face != other.left_face
		|| this->right_face != other.right_face) 
	{
		is_same = false;
	}

	return is_same;
}

bool Cube::operator!=(const Cube &other) const 
{
	return !(*this == other);
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
	
void Cube::rotate_front_face(int direction) 
{
	// check validity of argument
	if (direction != CW && direction != CCW) 
	{
		throw invalid_argument("Invalid direction: expected CW or CCW");
	}

	// rotate the face itself
	if (direction == CW) front_face.rotate_cw();
	else front_face.rotate_ccw();	

	// get the adjacent faces
	Face adjacent_left_face = *front_face.get_adjacent_left();		
	Face adjacent_right_face = *front_face.get_adjacent_right();		
	
	// save some cube state info
	vector<int> left_replacement_row = front_face.get_adjacent_bottom()->get_row1();
	vector<int> right_replacement_row = front_face.get_adjacent_top()->get_row3();
	if (direction == CCW)
	{
		left_replacement_row = reverse(front_face.get_adjacent_top()->get_row3());
		right_replacement_row = reverse(front_face.get_adjacent_bottom()->get_row1());
	}

	// fix the adjacent top face
	vector <int> new_top_column;
	if (direction == CW) new_top_column = get_column(adjacent_left_face, 2, true);
	else new_top_column = get_column(adjacent_right_face, 0, false);
	front_face.get_adjacent_top()->set_row3(new_top_column);

	// fix the adjacent bottom face
	vector<int> new_bottom_column;
	if (direction == CW) new_bottom_column = get_column(adjacent_right_face, 0, true);
	else new_bottom_column = get_column(adjacent_left_face, 2, false);
	front_face.get_adjacent_bottom()->set_row1(new_bottom_column);

	// fix the adjacent left face
	vector< vector<int> > fixed_left_face = fix_face(adjacent_left_face, 2, left_replacement_row, false);
	front_face.get_adjacent_left()->set_face(fixed_left_face);

	// fix the adjacent right face
	vector< vector<int> > fixed_right_face = fix_face(adjacent_right_face, 0, right_replacement_row, false);
	front_face.get_adjacent_right()->set_face(fixed_right_face);
}

////////////////////////////////////////////////////////////////////////////////

// Rotating Back Face
	
void Cube::rotate_back_face(int direction) 
{
	// check validity of argument
	if (direction != CW && direction != CCW) 
	{
		throw invalid_argument("Invalid direction: expected CW or CCW");
	}

	// rotate the face itself
	if (direction == CW) back_face.rotate_cw();
	else back_face.rotate_ccw();

	// get the adjacent faces
	Face adjacent_left_face = *back_face.get_adjacent_left();		
	Face adjacent_right_face = *back_face.get_adjacent_right();		
	
	// save some cube state info
	vector<int> left_replacement_row = back_face.get_adjacent_bottom()->get_row3();
	vector<int> right_replacement_row = back_face.get_adjacent_top()->get_row1();
	if (direction == CCW)
	{
		left_replacement_row = reverse(back_face.get_adjacent_top()->get_row1());
		right_replacement_row = reverse(back_face.get_adjacent_bottom()->get_row3());
	}

	// fix the adjacent top face
	vector<int> new_top_column;
	if (direction == CW) new_top_column = get_column(adjacent_left_face, 2, false);
	else new_top_column = get_column(adjacent_right_face, 0, true);
	back_face.get_adjacent_top()->set_row1(new_top_column);

	// fix the adjacent bottom face
	vector<int> new_bottom_column;
	if (direction == CW) new_bottom_column = get_column(adjacent_right_face, 0, false);
	else new_bottom_column = get_column(adjacent_left_face, 2, true);
	back_face.get_adjacent_bottom()->set_row3(new_bottom_column);

	// fix the adjacent left face
	vector< vector<int> > fixed_left_face = fix_face(adjacent_left_face, 2, left_replacement_row, true);
	back_face.get_adjacent_left()->set_face(fixed_left_face);

	// fix the adjacent right face
	vector< vector<int> > fixed_right_face = fix_face(adjacent_right_face, 0, right_replacement_row, true);
	back_face.get_adjacent_right()->set_face(fixed_right_face);
}

////////////////////////////////////////////////////////////////////////////////

// Rotating Top Face
	
void Cube::rotate_top_face(int direction) 
{
	// check validity of argument
	if (direction != CW && direction != CCW) 
	{
		throw invalid_argument("Invalid direction: expected CW or CCW");
	}

	// rotate the face itself
	if (direction == CW) top_face.rotate_cw();
	else top_face.rotate_ccw();
	
	// save some cube state info
	vector<int> left_replacement_row = top_face.get_adjacent_bottom()->get_row1();
	vector<int> right_replacement_row = top_face.get_adjacent_top()->get_row1();
	if (direction == CCW) 
	{
		left_replacement_row = top_face.get_adjacent_top()->get_row1();
		right_replacement_row = top_face.get_adjacent_bottom()->get_row1();
	}

	// fix top face
	if (direction == CW) top_face.get_adjacent_top()->set_row1(top_face.get_adjacent_left()->get_row1());
	else top_face.get_adjacent_top()->set_row1(top_face.get_adjacent_right()->get_row1());		

	// fix bottom face
	if (direction == CW) top_face.get_adjacent_bottom()->set_row1(top_face.get_adjacent_right()->get_row1());
	else top_face.get_adjacent_bottom()->set_row1(top_face.get_adjacent_left()->get_row1());		
	
	// fix left and right faces
	top_face.get_adjacent_left()->set_row1(left_replacement_row);
	top_face.get_adjacent_right()->set_row1(right_replacement_row);
}

////////////////////////////////////////////////////////////////////////////////

// Rotating Bottom Face
	
void Cube::rotate_bottom_face(int direction) 
{
	// check validity of argument
	if (direction != CW && direction != CCW) 
	{
		throw invalid_argument("Invalid direction: expected CW or CCW");
	}

	// rotate the face itself
	if (direction == CW) bottom_face.rotate_cw();
	else bottom_face.rotate_ccw();
	
	// save some cube state info
	vector<int> left_replacement_row = bottom_face.get_adjacent_bottom()->get_row3();
	vector<int> right_replacement_row = bottom_face.get_adjacent_top()->get_row3();
	if (direction == CCW) 
	{
		left_replacement_row = bottom_face.get_adjacent_top()->get_row3();
		right_replacement_row = bottom_face.get_adjacent_bottom()->get_row3();
	}

	// fix the top face
	if (direction == CW) bottom_face.get_adjacent_top()->set_row3(bottom_face.get_adjacent_left()->get_row3());
	else bottom_face.get_adjacent_top()->set_row3(top_face.get_adjacent_right()->get_row3());		

	// fix the bottom
	if (direction == CW) bottom_face.get_adjacent_bottom()->set_row3(bottom_face.get_adjacent_right()->get_row3());
	else bottom_face.get_adjacent_bottom()->set_row3(top_face.get_adjacent_left()->get_row3());		
	
	// fix left and right faces
	bottom_face.get_adjacent_left()->set_row3(left_replacement_row);
	bottom_face.get_adjacent_right()->set_row3(right_replacement_row);
}

////////////////////////////////////////////////////////////////////////////////

// Rotating Left Face
	
void Cube::rotate_left_face(int direction) 
{
	// check validity of argument
	if (direction != CW && direction != CCW) 
	{
		throw invalid_argument("Invalid direction: expected CW or CCW");
	}

	// rotate the face itself
	if (direction == CW) left_face.rotate_cw();
	else left_face.rotate_ccw();	

	// get the adjacent faces
	Face adjacent_left_face = *left_face.get_adjacent_left();		
	Face adjacent_right_face = *left_face.get_adjacent_right();		
	Face adjacent_top_face = *left_face.get_adjacent_top();		
	Face adjacent_bottom_face = *left_face.get_adjacent_bottom();		
	
	// save some cube state info
	vector<int> left_replacement_column = get_column(adjacent_bottom_face, 0, true);
	vector<int> right_replacement_column = get_column(adjacent_top_face, 0, false);
	vector<int> top_replacement_column = get_column(adjacent_left_face, 2, true);
	vector<int> bottom_replacement_column = get_column(adjacent_right_face, 0, false);
	if (direction == CCW) 
	{
		left_replacement_column = get_column(adjacent_top_face, 0, true);
		right_replacement_column = get_column(adjacent_bottom_face, 0, false);
		top_replacement_column = get_column(adjacent_right_face, 0, false);
		bottom_replacement_column = get_column(adjacent_left_face, 2, false);

	}

	// fix the adjacent top face
	vector< vector<int> > fixed_top_face;
	if (direction == CW) fixed_top_face = fix_face(adjacent_top_face, 0, top_replacement_column, false);
	else fixed_top_face = fix_face(adjacent_top_face, 0, top_replacement_column, false);
	left_face.get_adjacent_top()->set_face(fixed_top_face);

	// fix the adjacent bottom face
	vector< vector<int> > fixed_bottom_face;
	if (direction == CW) fixed_bottom_face = fix_face(adjacent_bottom_face, 0, bottom_replacement_column, false);
	else fixed_bottom_face = fix_face(adjacent_bottom_face, 0, bottom_replacement_column, true);
	left_face.get_adjacent_bottom()->set_face(fixed_bottom_face);

	// fix the adjacent left face
	vector< vector<int> > fixed_left_face;
	if (direction == CW) fixed_left_face = fix_face(adjacent_left_face, 2, left_replacement_column, false);
	else fixed_left_face = fix_face(adjacent_left_face, 2, left_replacement_column, false);
	left_face.get_adjacent_left()->set_face(fixed_left_face);

	// fix the adjacent right face
	vector< vector<int> > fixed_right_face;
	if (direction == CW) fixed_right_face = fix_face(adjacent_right_face, 0, right_replacement_column, false);
	else fixed_right_face = fix_face(adjacent_right_face, 0, right_replacement_column, false);
	left_face.get_adjacent_right()->set_face(fixed_right_face);
}

////////////////////////////////////////////////////////////////////////////////

// Rotating Right Face
	
void Cube::rotate_right_face(int direction) 
{
	// check validity of argument
	if (direction != CW && direction != CCW) 
	{
		throw invalid_argument("Invalid direction: expected CW or CCW");
	}

	// rotate the face itself
	if (direction == CW) right_face.rotate_cw();
	else right_face.rotate_ccw();	

	// get the adjacent faces
	Face adjacent_left_face = *right_face.get_adjacent_left();		
	Face adjacent_right_face = *right_face.get_adjacent_right();		
	Face adjacent_top_face = *right_face.get_adjacent_top();		
	Face adjacent_bottom_face = *right_face.get_adjacent_bottom();		
	
	// save some cube state info
	vector<int> left_replacement_column = get_column(adjacent_bottom_face, 2, false);
	vector<int> right_replacement_column = get_column(adjacent_top_face, 2, true);
	vector<int> top_replacement_column = get_column(adjacent_left_face, 2, true);
	vector<int> bottom_replacement_column  = get_column(adjacent_right_face, 0, false);
	if (direction == CCW) 
	{
		left_replacement_column = get_column(adjacent_top_face, 2, false);
		right_replacement_column = get_column(adjacent_bottom_face, 2, true);
		top_replacement_column = get_column(adjacent_right_face, 0, false);
		bottom_replacement_column = get_column(adjacent_left_face, 2, false);
	}

	// fix the adjacent top face
	vector< vector<int> > fixed_top_face;
	if (direction == CW) fixed_top_face = fix_face(adjacent_top_face, 2, top_replacement_column, true);
	else fixed_top_face = fix_face(adjacent_top_face, 2, top_replacement_column, true); 
	right_face.get_adjacent_top()->set_face(fixed_top_face);

	// fix the adjacent bottom face
	vector< vector<int> > fixed_bottom_face;
	if (direction == CW) fixed_bottom_face = fix_face(adjacent_bottom_face, 2, bottom_replacement_column, true);
	else fixed_bottom_face = fix_face(adjacent_bottom_face, 2, bottom_replacement_column, false);
	right_face.get_adjacent_bottom()->set_face(fixed_bottom_face);

	// fix the adjacent left face
	vector< vector<int> > fixed_left_face; 
	if (direction == CW) fixed_left_face = fix_face(adjacent_left_face, 2, left_replacement_column, false);
	else fixed_left_face = fix_face(adjacent_left_face, 2, left_replacement_column, false);
	right_face.get_adjacent_left()->set_face(fixed_left_face);

	// fix the adjacent right face
	vector< vector<int> > fixed_right_face;
	if (direction == CW) fixed_right_face = fix_face(adjacent_right_face, 0, right_replacement_column, false);
	else fixed_right_face = fix_face(adjacent_right_face, 0, right_replacement_column, false);
	right_face.get_adjacent_right()->set_face(fixed_right_face);
}

////////////////////////////////////////////////////////////////////////////////

// Helper Functions 

vector<int> Cube::get_column(Face face, int column, bool is_reversed) 
{
	vector<int> new_column;

	new_column.push_back(face.get_row1()[column]);
	new_column.push_back(face.get_row2()[column]);
	new_column.push_back(face.get_row3()[column]);

	if (is_reversed) return reverse(new_column);
	else return new_column;

}

vector< vector<int> > Cube::fix_face(Face face, int column, vector<int> replacement, bool is_reversed) 
{
	vector<int> new_row1 = face.get_row1();
	vector<int> new_row2 = face.get_row2();
	vector<int> new_row3 = face.get_row3();

	if (is_reversed) 
	{
		new_row1[column] = replacement[2];
		new_row2[column] = replacement[1];
		new_row3[column] = replacement[0];
	}

	else 
	{
		new_row1[column] = replacement[0];
		new_row2[column] = replacement[1];
		new_row3[column] = replacement[2];
	}

	vector< vector<int> > temp;
	temp.push_back(new_row1);
	temp.push_back(new_row2);
	temp.push_back(new_row3);

	return temp;
}

////////////////////////////////////////////////////////////////////////////////

// Handle Printing
	
void Cube::print() 
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

