///////////////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: main.cpp
// 
// Purpose: solves a rubiks cube
//
///////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <cstring>
#include <ctime>
#include "helper.hpp"
#include "face_class.hpp"
#include "cube_class.hpp"

#define GODS_NUM 7 // quarter turn metric

#define LEFT_CW 0
#define LEFT_CCW 1
#define RIGHT_CW 2
#define RIGHT_CCW 3
#define TOP_CW 4
#define TOP_CCW 5
#define BOTTOM_CW 6
#define BOTTOM_CCW 7
#define FRONT_CW 8
#define FRONT_CCW 9
#define BACK_CW 10
#define BACK_CCW 11

string solve_cube(Cube * passed_cube, int num_calls = 0);
vector<Face> make_faces();

Cube solved_cube(make_faces());

using namespace std;

int main() 
{
	// make a sample cube
	Cube mixed_cube(make_faces());
	mixed_cube.rotate_top_face(CW);
	mixed_cube.rotate_right_face(CW);
	mixed_cube.rotate_right_face(CW);
	mixed_cube.rotate_front_face(CW);
	mixed_cube.rotate_back_face(CW);
	mixed_cube.rotate_right_face(CW);
	mixed_cube.rotate_back_face(CW);
	/*mixed_cube.rotate_back_face(CW);
	mixed_cube.rotate_right_face(CW);
	mixed_cube.rotate_top_face(CW);
	mixed_cube.rotate_top_face(CW);
	mixed_cube.rotate_left_face(CW);
	mixed_cube.rotate_back_face(CW);
	mixed_cube.rotate_back_face(CW);
	mixed_cube.rotate_right_face(CW);
	mixed_cube.rotate_top_face(CCW);
	mixed_cube.rotate_bottom_face(CCW);
	mixed_cube.rotate_right_face(CW);
	mixed_cube.rotate_right_face(CW);
	mixed_cube.rotate_front_face(CW);
	mixed_cube.rotate_right_face(CCW);
	mixed_cube.rotate_left_face(CW);
	mixed_cube.rotate_bottom_face(CW);
	mixed_cube.rotate_bottom_face(CW);
	mixed_cube.rotate_top_face(CW);
	mixed_cube.rotate_top_face(CW);
	mixed_cube.rotate_front_face(CW);
	mixed_cube.rotate_front_face(CW);*/

	// solve the sample cube with only one face rotation
	clock_t t = clock();
	cout << solve_cube(&mixed_cube) << endl;
	t = clock() - t;
	cout << "computed in [" << ((float)t)/CLOCKS_PER_SEC << "] seconds" << endl;

	return 0;
}

///////////////////////////////////////////////////////////////////////////

// SOLVE CUBE FUNCTION
// recursively solves the passed rubik's cube
string solve_cube(Cube * passed_cube, int num_calls) 
{
	string temp = "";
	if (num_calls == GODS_NUM) return temp;
	cout << ++num_calls << endl;

	(*passed_cube).rotate_left_face(CW);
	if (*passed_cube == solved_cube) return "left_cw.";
	else if((temp = solve_cube(passed_cube, num_calls)).back() == '.') return "left_cw, " + temp;
	(*passed_cube).rotate_left_face(CCW);
		
	(*passed_cube).rotate_left_face(CCW);
	if (*passed_cube == solved_cube) return "left_ccw.";
	else if((temp = solve_cube(passed_cube, num_calls)).back() == '.') return "left_ccw, " + temp;
	(*passed_cube).rotate_left_face(CW);

	(*passed_cube).rotate_right_face(CW);
	if (*passed_cube == solved_cube) return "right_cw.";
	else if((temp = solve_cube(passed_cube, num_calls)).back() == '.') return "right_cw, " + temp;
	(*passed_cube).rotate_right_face(CCW);

	(*passed_cube).rotate_right_face(CCW);
	if (*passed_cube == solved_cube) return "right_ccw.";
	else if((temp = solve_cube(passed_cube, num_calls)).back() == '.') return "right_ccw, " + temp;
	(*passed_cube).rotate_right_face(CW);

	(*passed_cube).rotate_top_face(CW);
	if (*passed_cube == solved_cube) return "top_cw.";
	else if((temp = solve_cube(passed_cube, num_calls)).back() == '.') return "top_cw, " + temp;
	(*passed_cube).rotate_top_face(CCW);

	(*passed_cube).rotate_top_face(CCW);
	if (*passed_cube == solved_cube) return "top_ccw.";
	else if((temp = solve_cube(passed_cube, num_calls)).back() == '.') return "top_ccw, " + temp;
	(*passed_cube).rotate_top_face(CW);

	(*passed_cube).rotate_bottom_face(CW);
	if (*passed_cube == solved_cube) return "bottom_cw.";
	else if((temp = solve_cube(passed_cube, num_calls)).back() == '.') return "bottom_cw, " + temp;
	(*passed_cube).rotate_bottom_face(CCW);

	(*passed_cube).rotate_bottom_face(CCW);
	if (*passed_cube == solved_cube) return "bottom_ccw.";
	else if((temp = solve_cube(passed_cube, num_calls)).back() == '.') return "bottom_ccw, " + temp;
	(*passed_cube).rotate_bottom_face(CW);

	(*passed_cube).rotate_front_face(CW);
	if (*passed_cube == solved_cube) return "front_cw.";
	else if((temp = solve_cube(passed_cube, num_calls)).back() == '.') return "front_cw, " + temp;
	(*passed_cube).rotate_front_face(CCW);

	(*passed_cube).rotate_front_face(CCW);
	if (*passed_cube == solved_cube) return "front_ccw.";
	else if((temp = solve_cube(passed_cube, num_calls)).back() == '.') return "front_ccw, " + temp;
	(*passed_cube).rotate_front_face(CW);

	(*passed_cube).rotate_back_face(CW);
	if (*passed_cube == solved_cube) return "back_cw.";
	else if((temp = solve_cube(passed_cube, num_calls)).back() == '.') return "back_cw, " + temp;
	(*passed_cube).rotate_back_face(CCW);

	(*passed_cube).rotate_back_face(CCW);
	if (*passed_cube == solved_cube) return "back_ccw.";
	else if((temp = solve_cube(passed_cube, num_calls)).back() == '.') return "back_ccw, " + temp;
	(*passed_cube).rotate_back_face(CW);

	return "";
}

// MAKE FACES FUNCTION
// makes a solved rubik's cube
vector<Face> make_faces() 
{
	vector<Face> faces;
	vector< vector<int> > face_color_matrix;
	vector<int> temp_row1;
	vector<int> temp_row2;
	vector<int> temp_row3;
						
	temp_row1.push_back(RED);
	temp_row1.push_back(RED);
	temp_row1.push_back(RED);
	
	temp_row2.push_back(RED);
	temp_row2.push_back(RED);
	temp_row2.push_back(RED);
	
	temp_row3.push_back(RED);
	temp_row3.push_back(RED);
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
	temp_row1.push_back(ORANGE);
	
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(ORANGE);
	temp_row2.push_back(ORANGE);
	
	temp_row3.push_back(ORANGE);
	temp_row3.push_back(ORANGE);
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

	temp_row1.push_back(YELLOW);
	temp_row1.push_back(YELLOW);
	temp_row1.push_back(YELLOW);
	
	temp_row2.push_back(YELLOW);
	temp_row2.push_back(YELLOW);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(YELLOW);
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
	temp_row1.push_back(WHITE);
	temp_row1.push_back(WHITE);
	
	temp_row2.push_back(WHITE);
	temp_row2.push_back(WHITE);
	temp_row2.push_back(WHITE);
	
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

	temp_row1.push_back(BLUE);
	temp_row1.push_back(BLUE);
	temp_row1.push_back(BLUE);
	
	temp_row2.push_back(BLUE);
	temp_row2.push_back(BLUE);
	temp_row2.push_back(BLUE);
	
	temp_row3.push_back(BLUE);
	temp_row3.push_back(BLUE);
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

	temp_row1.push_back(GREEN);
	temp_row1.push_back(GREEN);
	temp_row1.push_back(GREEN);
	
	temp_row2.push_back(GREEN);
	temp_row2.push_back(GREEN);
	temp_row2.push_back(GREEN);
	
	temp_row3.push_back(GREEN);
	temp_row3.push_back(GREEN);
	temp_row3.push_back(GREEN);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	Face right(face_color_matrix);
	faces.push_back(right);
	
	return faces;
}

///////////////////////////////////////////////////////////////////////////

