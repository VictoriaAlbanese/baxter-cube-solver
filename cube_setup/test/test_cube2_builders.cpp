
////////////////////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// File name: test_cube2_builders.cpp
// Purpose: Helper functions which each manually create a solved cube with 
// one different turn to test against.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef TEST_CUBE2_BUILDERS_CPP
#define TEST_CUBE2_BUILDERS_CPP

#include "../src/helper.hpp"
#include "../src/face_class.hpp"

////////////////////////////////////////////////////////////////////////////////
//
// Make Test Face Vector
// Arguments: n/a
// Returns: A vector of faces which creates a test cube
//
////////////////////////////////////////////////////////////////////////////////

static vector<Face> make_test_face_vector2() 
{
	vector<Face> faces;
	vector< vector<int> > face_color_matrix;
	vector<int> temp_row1;
	vector<int> temp_row2;
	vector<int> temp_row3;
						
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(YELLOW);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(RED);
	temp_row2.push_back(GREEN);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face front(face_color_matrix);
	faces.push_back(front);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(WHITE);
	temp_row1.push_back(GREEN);
	temp_row1.push_back(YELLOW);
	
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(BLUE);
	temp_row3.push_back(RED);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face back(face_color_matrix);
	faces.push_back(back);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(BLUE);
	temp_row1.push_back(WHITE);
	temp_row1.push_back(RED);
	
	temp_row2.push_back(WHITE);
	temp_row2.push_back(YELLOW);
	temp_row2.push_back(WHITE);
	
	temp_row3.push_back(GREEN);
	temp_row3.push_back(ORANGE);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face top(face_color_matrix);
	faces.push_back(top);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(WHITE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(WHITE);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face bottom(face_color_matrix);
	faces.push_back(bottom);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(BLUE);
	temp_row1.push_back(WHITE);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(BLUE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);
	temp_row3.push_back(BLUE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face left(face_color_matrix);
	faces.push_back(left);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(BLUE);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(GREEN);
	temp_row2.push_back(BLUE);
	
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face right(face_color_matrix);
	faces.push_back(right);
	
	return faces;
}

////////////////////////////////////////////////////////////////////////////////
//
// Make Test Face Vector front CW
// Arguments: n/a
// Returns: A vector of faces which creates a test cube with the front face 
// having been rotated clockwise one quarter turn
//
////////////////////////////////////////////////////////////////////////////////

static vector<Face> make_test_face_vector2_front_cw() 
{
	vector<Face> faces;
	vector< vector<int> > face_color_matrix;
	vector<int> temp_row1;
	vector<int> temp_row2;
	vector<int> temp_row3;
						
	temp_row1.push_back(WHITE);
	temp_row1.push_back(BLUE);
	temp_row1.push_back(ORANGE);
	
	temp_row2.push_back(GREEN);
	temp_row2.push_back(RED);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(RED);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(GREEN);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face front(face_color_matrix);
	faces.push_back(front);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(WHITE);
	temp_row1.push_back(GREEN);
	temp_row1.push_back(YELLOW);
	
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(BLUE);
	temp_row3.push_back(RED);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face back(face_color_matrix);
	faces.push_back(back);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(BLUE);
	temp_row1.push_back(WHITE);
	temp_row1.push_back(RED);
	
	temp_row2.push_back(WHITE);
	temp_row2.push_back(YELLOW);
	temp_row2.push_back(WHITE);
	
	temp_row3.push_back(BLUE);
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(WHITE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face top(face_color_matrix);
	faces.push_back(top);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(YELLOW);
	temp_row1.push_back(RED);
	temp_row1.push_back(ORANGE);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(WHITE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(WHITE);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face bottom(face_color_matrix);
	faces.push_back(bottom);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(BLUE);
	temp_row1.push_back(ORANGE);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(BLUE);
	temp_row2.push_back(ORANGE);
	
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);
	temp_row3.push_back(GREEN);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face left(face_color_matrix);
	faces.push_back(left);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(GREEN);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(BLUE);
	
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(GREEN);
	temp_row2.push_back(BLUE);
	
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face right(face_color_matrix);
	faces.push_back(right);
	
	return faces;
}

////////////////////////////////////////////////////////////////////////////////
//
// Make Test Face Vector Front CCW
// Arguments: n/a
// Returns: A vector of faces which creates a test cube with a front face 
// having been turned counterclockwise one quarter turn 
//
////////////////////////////////////////////////////////////////////////////////

static vector<Face> make_test_face_vector2_front_ccw() 
{
	vector<Face> faces;
	vector< vector<int> > face_color_matrix;
	vector<int> temp_row1;
	vector<int> temp_row2;
	vector<int> temp_row3;
						
	temp_row1.push_back(GREEN);
	temp_row1.push_back(GREEN);
	temp_row1.push_back(RED);
	
	temp_row2.push_back(YELLOW);
	temp_row2.push_back(RED);
	temp_row2.push_back(GREEN);
	
	temp_row3.push_back(ORANGE);
	temp_row3.push_back(BLUE);
	temp_row3.push_back(WHITE);
	
	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face front(face_color_matrix);
	faces.push_back(front);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(WHITE);
	temp_row1.push_back(GREEN);
	temp_row1.push_back(YELLOW);
	
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(BLUE);
	temp_row3.push_back(RED);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face back(face_color_matrix);
	faces.push_back(back);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(BLUE);
	temp_row1.push_back(WHITE);
	temp_row1.push_back(RED);
	
	temp_row2.push_back(WHITE);
	temp_row2.push_back(YELLOW);
	temp_row2.push_back(WHITE);
	
	temp_row3.push_back(ORANGE);
	temp_row3.push_back(RED);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face top(face_color_matrix);
	faces.push_back(top);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(WHITE);
	temp_row1.push_back(YELLOW);
	temp_row1.push_back(BLUE);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(WHITE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(WHITE);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face bottom(face_color_matrix);
	faces.push_back(bottom);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(BLUE);
	temp_row1.push_back(YELLOW);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(BLUE);
	temp_row2.push_back(ORANGE);
	
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);
	temp_row3.push_back(GREEN);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face left(face_color_matrix);
	faces.push_back(left);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(GREEN);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(BLUE);
	
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(GREEN);
	temp_row2.push_back(BLUE);
	
	temp_row3.push_back(ORANGE);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face right(face_color_matrix);
	faces.push_back(right);
	
	return faces;
}

////////////////////////////////////////////////////////////////////////////////
//
// Make Test Face Vector Back CW
// Arguments: n/a
// Returns: A vector of faces which creates a test cube with a back face 
// having been turned clockwise one quarter turn
//
////////////////////////////////////////////////////////////////////////////////

static vector<Face> make_test_face_vector2_back_cw() 
{
	vector<Face> faces;
	vector< vector<int> > face_color_matrix;
	vector<int> temp_row1;
	vector<int> temp_row2;
	vector<int> temp_row3;
						
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(YELLOW);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(RED);
	temp_row2.push_back(GREEN);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face front(face_color_matrix);
	faces.push_back(front);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(BLUE);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(WHITE);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(GREEN);
	
	temp_row3.push_back(RED);
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face back(face_color_matrix);
	faces.push_back(back);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(BLUE);
	temp_row1.push_back(BLUE);
	temp_row1.push_back(RED);
	
	temp_row2.push_back(WHITE);
	temp_row2.push_back(YELLOW);
	temp_row2.push_back(WHITE);
	
	temp_row3.push_back(GREEN);
	temp_row3.push_back(ORANGE);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face top(face_color_matrix);
	faces.push_back(top);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(WHITE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(ORANGE);
	temp_row3.push_back(RED);
	temp_row3.push_back(GREEN);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face bottom(face_color_matrix);
	faces.push_back(bottom);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(RED);
	temp_row1.push_back(BLUE);
	temp_row1.push_back(WHITE);
	
	temp_row2.push_back(WHITE);
	temp_row2.push_back(BLUE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(BLUE);
	temp_row3.push_back(RED);
	temp_row3.push_back(BLUE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face left(face_color_matrix);
	faces.push_back(left);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(YELLOW);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(GREEN);
	temp_row2.push_back(WHITE);
	
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(WHITE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face right(face_color_matrix);
	faces.push_back(right);
	
	return faces;
}

////////////////////////////////////////////////////////////////////////////////
//
// Make Test Face Vector Back CCW
// Arguments: n/a
// Returns: A vector of faces which creates a test cube with a back face
// having been turned counterclockwise one quarter turn
//
////////////////////////////////////////////////////////////////////////////////

static vector<Face> make_test_face_vector2_back_ccw() 
{
	vector<Face> faces;
	vector< vector<int> > face_color_matrix;
	vector<int> temp_row1;
	vector<int> temp_row2;
	vector<int> temp_row3;
						
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(YELLOW);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(RED);
	temp_row2.push_back(GREEN);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face front(face_color_matrix);
	faces.push_back(front);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(YELLOW);
	temp_row1.push_back(YELLOW);
	temp_row1.push_back(RED);
	
	temp_row2.push_back(GREEN);
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(RED);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(ORANGE);
	temp_row3.push_back(BLUE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face back(face_color_matrix);
	faces.push_back(back);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(GREEN);
	temp_row1.push_back(RED);
	temp_row1.push_back(ORANGE);
	
	temp_row2.push_back(WHITE);
	temp_row2.push_back(YELLOW);
	temp_row2.push_back(WHITE);
	
	temp_row3.push_back(GREEN);
	temp_row3.push_back(ORANGE);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face top(face_color_matrix);
	faces.push_back(top);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(WHITE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(RED);
	temp_row3.push_back(BLUE);
	temp_row3.push_back(BLUE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face bottom(face_color_matrix);
	faces.push_back(bottom);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(WHITE);
	temp_row1.push_back(BLUE);
	temp_row1.push_back(WHITE);
	
	temp_row2.push_back(WHITE);
	temp_row2.push_back(BLUE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(RED);
	temp_row3.push_back(BLUE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face left(face_color_matrix);
	faces.push_back(left);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(BLUE);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(GREEN);
	temp_row2.push_back(WHITE);
	
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face right(face_color_matrix);
	faces.push_back(right);
	
	return faces;
}

////////////////////////////////////////////////////////////////////////////////
//
// Make Test Face Vector Top CW
// Arguments: n/a
// Returns: A vector of faces which creates a test cube with the top face
// having been rotated clockwise one quarter turn
//
////////////////////////////////////////////////////////////////////////////////

static vector<Face> make_test_face_vector2_top_cw() 
{
	vector<Face> faces;
	vector< vector<int> > face_color_matrix;
	vector<int> temp_row1;
	vector<int> temp_row2;
	vector<int> temp_row3;
						
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(BLUE);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(RED);
	temp_row2.push_back(GREEN);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face front(face_color_matrix);
	faces.push_back(front);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(BLUE);
	temp_row1.push_back(WHITE);
	
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(BLUE);
	temp_row3.push_back(RED);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face back(face_color_matrix);
	faces.push_back(back);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(GREEN);
	temp_row1.push_back(WHITE);
	temp_row1.push_back(BLUE);
	
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(YELLOW);
	temp_row2.push_back(WHITE);
	
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(WHITE);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face top(face_color_matrix);
	faces.push_back(top);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(WHITE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(WHITE);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face bottom(face_color_matrix);
	faces.push_back(bottom);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(YELLOW);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(BLUE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);
	temp_row3.push_back(BLUE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face left(face_color_matrix);
	faces.push_back(left);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(WHITE);
	temp_row1.push_back(GREEN);
	temp_row1.push_back(YELLOW);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(GREEN);
	temp_row2.push_back(BLUE);
	
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face right(face_color_matrix);
	faces.push_back(right);
	
	return faces;
}

////////////////////////////////////////////////////////////////////////////////
//
// Make Test Face Vector Top CCW
// Arguments: n/a
// Returns: A vector of faces which creates a test cube with a top face
// having been rotated counterclockwise by one quarter turn
//
////////////////////////////////////////////////////////////////////////////////

static vector<Face> make_test_face_vector2_top_ccw() 
{
	vector<Face> faces;
	vector< vector<int> > face_color_matrix;
	vector<int> temp_row1;
	vector<int> temp_row2;
	vector<int> temp_row3;
						
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(BLUE);
	temp_row1.push_back(WHITE);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(RED);
	temp_row2.push_back(GREEN);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face front(face_color_matrix);
	faces.push_back(front);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(BLUE);
	
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(BLUE);
	temp_row3.push_back(RED);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face back(face_color_matrix);
	faces.push_back(back);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(RED);
	temp_row1.push_back(WHITE);
	temp_row1.push_back(YELLOW);
	
	temp_row2.push_back(WHITE);
	temp_row2.push_back(YELLOW);
	temp_row2.push_back(ORANGE);
	
	temp_row3.push_back(BLUE);
	temp_row3.push_back(WHITE);
	temp_row3.push_back(GREEN);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face top(face_color_matrix);
	faces.push_back(top);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(WHITE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(WHITE);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face bottom(face_color_matrix);
	faces.push_back(bottom);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(WHITE);
	temp_row1.push_back(GREEN);
	temp_row1.push_back(YELLOW);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(BLUE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);
	temp_row3.push_back(BLUE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face left(face_color_matrix);
	faces.push_back(left);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(YELLOW);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(GREEN);
	temp_row2.push_back(BLUE);
	
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face right(face_color_matrix);
	faces.push_back(right);
	
	return faces;
}

////////////////////////////////////////////////////////////////////////////////
//
// Make Test Face Vector Bottom CW
// Arguments: n/a
// Returns: A vector of faces which creates a test cube with the bottom face 
// having been rotated clockwise one quarter turn
//
////////////////////////////////////////////////////////////////////////////////

static vector<Face> make_test_face_vector2_bottom_cw() 
{
	vector<Face> faces;
	vector< vector<int> > face_color_matrix;
	vector<int> temp_row1;
	vector<int> temp_row2;
	vector<int> temp_row3;
						
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(YELLOW);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(RED);
	temp_row2.push_back(GREEN);
	
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);
	temp_row3.push_back(BLUE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face front(face_color_matrix);
	faces.push_back(front);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(WHITE);
	temp_row1.push_back(GREEN);
	temp_row1.push_back(YELLOW);
	
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face back(face_color_matrix);
	faces.push_back(back);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(BLUE);
	temp_row1.push_back(WHITE);
	temp_row1.push_back(RED);
	
	temp_row2.push_back(WHITE);
	temp_row2.push_back(YELLOW);
	temp_row2.push_back(WHITE);
	
	temp_row3.push_back(GREEN);
	temp_row3.push_back(ORANGE);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face top(face_color_matrix);
	faces.push_back(top);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(WHITE);
	temp_row1.push_back(BLUE);
	temp_row1.push_back(ORANGE);
	
	temp_row2.push_back(WHITE);
	temp_row2.push_back(WHITE);
	temp_row2.push_back(ORANGE);
	
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(GREEN);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face bottom(face_color_matrix);
	faces.push_back(bottom);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(BLUE);
	temp_row1.push_back(WHITE);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(BLUE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(BLUE);
	temp_row3.push_back(RED);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face left(face_color_matrix);
	faces.push_back(left);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(BLUE);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(GREEN);
	temp_row2.push_back(BLUE);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face right(face_color_matrix);
	faces.push_back(right);
	
	return faces;
}

////////////////////////////////////////////////////////////////////////////////
//
// Make Test Face Vector Bottom CCW
// Arguments: n/a
// Returns: A vector of faces which creates a test cube with a bottom face
// having been rotated counterclockwise one quarter turn
//
////////////////////////////////////////////////////////////////////////////////

static vector<Face> make_test_face_vector2_bottom_ccw() 
{
	vector<Face> faces;
	vector< vector<int> > face_color_matrix;
	vector<int> temp_row1;
	vector<int> temp_row2;
	vector<int> temp_row3;
						
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(YELLOW);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(RED);
	temp_row2.push_back(GREEN);
	
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face front(face_color_matrix);
	faces.push_back(front);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(WHITE);
	temp_row1.push_back(GREEN);
	temp_row1.push_back(YELLOW);
	
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);
	temp_row3.push_back(BLUE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face back(face_color_matrix);
	faces.push_back(back);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(BLUE);
	temp_row1.push_back(WHITE);
	temp_row1.push_back(RED);
	
	temp_row2.push_back(WHITE);
	temp_row2.push_back(YELLOW);
	temp_row2.push_back(WHITE);
	
	temp_row3.push_back(GREEN);
	temp_row3.push_back(ORANGE);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face top(face_color_matrix);
	faces.push_back(top);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(GREEN);
	temp_row1.push_back(YELLOW);
	temp_row1.push_back(YELLOW);
	
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(WHITE);
	temp_row2.push_back(WHITE);
	
	temp_row3.push_back(ORANGE);
	temp_row3.push_back(BLUE);
	temp_row3.push_back(WHITE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face bottom(face_color_matrix);
	faces.push_back(bottom);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(BLUE);
	temp_row1.push_back(WHITE);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(BLUE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face left(face_color_matrix);
	faces.push_back(left);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(BLUE);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(GREEN);
	temp_row2.push_back(BLUE);
	
	temp_row3.push_back(BLUE);
	temp_row3.push_back(RED);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face right(face_color_matrix);
	faces.push_back(right);
	
	return faces;
}

////////////////////////////////////////////////////////////////////////////////
//
// Make Test Face Vector Left CW
// Arguments: n/a
// Returns: A vector of faces which creates a test cube with a left face
// having been rttated clockwise one quarter turn
//
////////////////////////////////////////////////////////////////////////////////

static vector<Face> make_test_face_vector2_left_cw() 
{
	vector<Face> faces;
	vector< vector<int> > face_color_matrix;
	vector<int> temp_row1;
	vector<int> temp_row2;
	vector<int> temp_row3;
						
	temp_row1.push_back(BLUE);
	temp_row1.push_back(YELLOW);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(WHITE);
	temp_row2.push_back(RED);
	temp_row2.push_back(GREEN);
	
	temp_row3.push_back(GREEN);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face front(face_color_matrix);
	faces.push_back(front);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(WHITE);
	temp_row1.push_back(GREEN);
	temp_row1.push_back(WHITE);
	
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(BLUE);
	
	temp_row3.push_back(BLUE);
	temp_row3.push_back(RED);
	temp_row3.push_back(ORANGE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face back(face_color_matrix);
	faces.push_back(back);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(RED);
	temp_row1.push_back(WHITE);
	temp_row1.push_back(RED);
	
	temp_row2.push_back(YELLOW);
	temp_row2.push_back(YELLOW);
	temp_row2.push_back(WHITE);
	
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(ORANGE);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face top(face_color_matrix);
	faces.push_back(top);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(WHITE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(WHITE);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face bottom(face_color_matrix);
	faces.push_back(bottom);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(GREEN);
	temp_row1.push_back(RED);
	temp_row1.push_back(ORANGE);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(BLUE);
	temp_row2.push_back(BLUE);
	
	temp_row3.push_back(BLUE);
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(WHITE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face left(face_color_matrix);
	faces.push_back(left);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(BLUE);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(GREEN);
	temp_row2.push_back(BLUE);
	
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face right(face_color_matrix);
	faces.push_back(right);
	
	return faces;
}

////////////////////////////////////////////////////////////////////////////////
//
// Make Test Face Vector Left CCW
// Arguments: n/a
// Returns: A vector of faces which creates a test cube with the left face 
// having been rotated counterclockwise one quarter turn
//
////////////////////////////////////////////////////////////////////////////////

static vector<Face> make_test_face_vector2_left_ccw() 
{
	vector<Face> faces;
	vector< vector<int> > face_color_matrix;
	vector<int> temp_row1;
	vector<int> temp_row2;
	vector<int> temp_row3;
						
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(YELLOW);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(RED);
	temp_row2.push_back(GREEN);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face front(face_color_matrix);
	faces.push_back(front);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(WHITE);
	temp_row1.push_back(GREEN);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(WHITE);
	
	temp_row3.push_back(BLUE);
	temp_row3.push_back(RED);
	temp_row3.push_back(BLUE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face back(face_color_matrix);
	faces.push_back(back);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(WHITE);
	temp_row1.push_back(RED);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(YELLOW);
	temp_row2.push_back(WHITE);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(ORANGE);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face top(face_color_matrix);
	faces.push_back(top);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(RED);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(YELLOW);
	temp_row2.push_back(WHITE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(WHITE);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face bottom(face_color_matrix);
	faces.push_back(bottom);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(WHITE);
	temp_row1.push_back(YELLOW);
	temp_row1.push_back(BLUE);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(BLUE);
	temp_row2.push_back(RED);
	
	temp_row3.push_back(ORANGE);
	temp_row3.push_back(RED);
	temp_row3.push_back(GREEN);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face left(face_color_matrix);
	faces.push_back(left);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(BLUE);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(GREEN);
	temp_row2.push_back(BLUE);
	
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face right(face_color_matrix);
	faces.push_back(right);
	
	return faces;
}

////////////////////////////////////////////////////////////////////////////////
//
// Make Test Face Vector Right CW
// Arguments: n/a
// Returns: A vector of faces which creates a test cube with the right face 
// having been rotated clockwise one quarter turn
//
////////////////////////////////////////////////////////////////////////////////

static vector<Face> make_test_face_vector2_right_cw() 
{
	vector<Face> faces;
	vector< vector<int> > face_color_matrix;
	vector<int> temp_row1;
	vector<int> temp_row2;
	vector<int> temp_row3;
						
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(YELLOW);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(RED);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face front(face_color_matrix);
	faces.push_back(front);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();
	
	temp_row1.push_back(YELLOW);
	temp_row1.push_back(GREEN);
	temp_row1.push_back(YELLOW);
	
	temp_row2.push_back(WHITE);
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(YELLOW);

	temp_row3.push_back(RED);
	temp_row3.push_back(RED);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face back(face_color_matrix);
	faces.push_back(back);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(BLUE);
	temp_row1.push_back(WHITE);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(WHITE);
	temp_row2.push_back(YELLOW);
	temp_row2.push_back(GREEN);
	
	temp_row3.push_back(GREEN);
	temp_row3.push_back(ORANGE);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face top(face_color_matrix);
	faces.push_back(top);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(BLUE);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(WHITE);
	temp_row2.push_back(ORANGE);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(WHITE);
	temp_row3.push_back(WHITE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face bottom(face_color_matrix);
	faces.push_back(bottom);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(BLUE);
	temp_row1.push_back(WHITE);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(BLUE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);
	temp_row3.push_back(BLUE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face left(face_color_matrix);
	faces.push_back(left);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(YELLOW);
	temp_row1.push_back(RED);
	temp_row1.push_back(ORANGE);
	
	temp_row2.push_back(GREEN);
	temp_row2.push_back(GREEN);
	temp_row2.push_back(ORANGE);
	
	temp_row3.push_back(RED);
	temp_row3.push_back(BLUE);
	temp_row3.push_back(BLUE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face right(face_color_matrix);
	faces.push_back(right);
	
	return faces;
}

////////////////////////////////////////////////////////////////////////////////
//
// Make Test Face Vector Right CCW
// Arguments: n/a
// Returns: A vector of faces which creates a test cube with right face
// having been rotated counterclockwise one quarter turn
//
////////////////////////////////////////////////////////////////////////////////

static vector<Face> make_test_face_vector2_right_ccw() 
{
	vector<Face> faces;
	vector< vector<int> > face_color_matrix;
	vector<int> temp_row1;
	vector<int> temp_row2;
	vector<int> temp_row3;
						
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(YELLOW);
	temp_row1.push_back(RED);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(RED);
	temp_row2.push_back(WHITE);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face front(face_color_matrix);
	faces.push_back(front);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(YELLOW);
	temp_row1.push_back(GREEN);
	temp_row1.push_back(YELLOW);
	
	temp_row2.push_back(YELLOW);
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face back(face_color_matrix);
	faces.push_back(back);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(BLUE);
	temp_row1.push_back(WHITE);
	temp_row1.push_back(BLUE);
	
	temp_row2.push_back(WHITE);
	temp_row2.push_back(YELLOW);
	temp_row2.push_back(ORANGE);
	
	temp_row3.push_back(GREEN);
	temp_row3.push_back(ORANGE);
	temp_row3.push_back(WHITE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face top(face_color_matrix);
	faces.push_back(top);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(WHITE);
	temp_row2.push_back(GREEN);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(WHITE);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face bottom(face_color_matrix);
	faces.push_back(bottom);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(ORANGE);
	temp_row1.push_back(BLUE);
	temp_row1.push_back(WHITE);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(BLUE);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(GREEN);
	temp_row3.push_back(RED);
	temp_row3.push_back(BLUE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face left(face_color_matrix);
	faces.push_back(left);

	face_color_matrix.clear();
	temp_row1.clear();
	temp_row2.clear();
	temp_row3.clear();

	temp_row1.push_back(BLUE);
	temp_row1.push_back(BLUE);
	temp_row1.push_back(RED);
	
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(GREEN);
	temp_row2.push_back(GREEN);
	
	temp_row3.push_back(ORANGE);
	temp_row3.push_back(RED);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face right(face_color_matrix);
	faces.push_back(right);
	
	return faces;
}

#endif // TEST_CUBE2_BUILDERS_CPP

////////////////////////////////////////////////////////////////////////////////

