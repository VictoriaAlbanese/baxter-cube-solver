
////////////////////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// File name: test.cpp
// Purpose: Runs tests
//
////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include "helper.hpp"
#include "face_class.hpp"
#include "cube_class.hpp"

vector< vector<int> > make_test_color_matrix();
vector<Face> make_test_face_vector(); 
void DefaultConstructor_CalledNormally_BlankFaceCreated();
void Constructor_CalledNormally_CorrectFaceCreated();
void RotateCW_CalledNormally_FaceIsRotatedCorrectly(); 
void RotateCCW_CalledNormally_FaceIsRotatedCorrectly(); 
void DefaultConstructor_CalledNormally_BlankCubeCreated();
void Constructor_CalledNormally_CorrectCubeCreated();
void RotateFrontCW_CalledNormally_CubeIsRotatedCorrectly(); 
void RotateFrontCCW_CalledNormally_CubeIsRotatedCorrectly(); 
void RotateBackCW_CalledNormally_CubeIsRotatedCorrectly(); 
void RotateBackCCW_CalledNormally_CubeIsRotatedCorrectly(); 
void RotateTopCW_CalledNormally_CubeIsRotatedCorrectly(); 
void RotateTopCCW_CalledNormally_CubeIsRotatedCorrectly(); 
void RotateBottomCW_CalledNormally_CubeIsRotatedCorrectly(); 
void RotateBottomCCW_CalledNormally_CubeIsRotatedCorrectly(); 
void RotateLeftCW_CalledNormally_CubeIsRotatedCorrectly(); 
void RotateLeftCCW_CalledNormally_CubeIsRotatedCorrectly(); 
void RotateRightCW_CalledNormally_CubeIsRotatedCorrectly(); 
void RotateRightCCW_CalledNormally_CubeIsRotatedCorrectly(); 

using namespace std;

int main() {

	// Face Testing
	//DefaultConstructor_CalledNormally_BlankFaceCreated();
	//Constructor_CalledNormally_CorrectFaceCreated();
	//RotateCW_CalledNormally_FaceIsRotatedCorrectly(); 
	//RotateCCW_CalledNormally_FaceIsRotatedCorrectly(); 

	// Cube Testing
	//DefaultConstructor_CalledNormally_BlankCubeCreated();
	Constructor_CalledNormally_CorrectCubeCreated();
	//RotateFrontCW_CalledNormally_CubeIsRotatedCorrectly(); 
	//RotateFrontCCW_CalledNormally_CubeIsRotatedCorrectly(); 
	//RotateBackCW_CalledNormally_CubeIsRotatedCorrectly(); 
	//RotateBackCCW_CalledNormally_CubeIsRotatedCorrectly(); 
	//RotateTopCW_CalledNormally_CubeIsRotatedCorrectly(); 
	//RotateTopCCW_CalledNormally_CubeIsRotatedCorrectly(); 
	//RotateBottomCW_CalledNormally_CubeIsRotatedCorrectly(); 
	//RotateBottomCCW_CalledNormally_CubeIsRotatedCorrectly(); 
	//RotateLeftCW_CalledNormally_CubeIsRotatedCorrectly(); 
	//RotateLeftCCW_CalledNormally_CubeIsRotatedCorrectly(); 
	RotateRightCW_CalledNormally_CubeIsRotatedCorrectly(); 
	RotateRightCCW_CalledNormally_CubeIsRotatedCorrectly(); 

	return 0;

}
	

////////////////////////////////////////////////////////////////////////////////

// Face Testing

void DefaultConstructor_CalledNormally_BlankFaceCreated() 
{
	// Create blank face
	Face blank_face;

	// Print face for manual checking
	blank_face.print_face();
}

void Constructor_CalledNormally_CorrectFaceCreated() 
{
	// Create mixed face
	vector< vector<int> > face_color_matrix = make_test_color_matrix();
	Face interesting_face(face_color_matrix);

	// Print face for manual checking
	interesting_face.print_face();
}	

void RotateCW_CalledNormally_FaceIsRotatedCorrectly() 
{
	// Create mixed face
	vector< vector<int> > face_color_matrix = make_test_color_matrix();
	Face interesting_face(face_color_matrix);

	// Rotate the face
	interesting_face.rotate_cw();

	// Print face for manual checking
	interesting_face.print_face();
}

void RotateCCW_CalledNormally_FaceIsRotatedCorrectly() 
{
	// Create a mixed face
	vector< vector<int> > face_color_matrix = make_test_color_matrix();
	Face interesting_face(face_color_matrix);

	// Rotate the face
	interesting_face.rotate_ccw();

	// Print face for manual checking
	interesting_face.print_face();
}

////////////////////////////////////////////////////////////////////////////////

// Cube Testing

void DefaultConstructor_CalledNormally_BlankCubeCreated() 
{
	// Create a blank cube
	Cube solved_cube;

	// Print face for manual checking
	solved_cube.print_cube();
}

void Constructor_CalledNormally_CorrectCubeCreated() 
{
	// Create a mixed cube
	vector<Face> faces = make_test_face_vector();
	Cube interesting_face_cube(faces);

	// Print face for manual checking
	interesting_face_cube.print_cube();
}

void RotateFrontCW_CalledNormally_CubeIsRotatedCorrectly() 
{
	// Create a mixed cube
	vector<Face> faces = make_test_face_vector();
	Cube interesting_face_cube(faces);

	// Rotate the front face
	interesting_face_cube.rotate_front_face_cw();

	// Print face for manual checking
	interesting_face_cube.print_cube();
}

void RotateFrontCCW_CalledNormally_CubeIsRotatedCorrectly() 
{
	// Create a mixed cube
	vector<Face> faces = make_test_face_vector();
	Cube interesting_face_cube(faces);

	// Rotate the front face
	interesting_face_cube.rotate_front_face_ccw();

	// Print face for manual checking
	interesting_face_cube.print_cube();
}

void RotateBackCW_CalledNormally_CubeIsRotatedCorrectly() 
{
	// Create a mixed cube
	vector<Face> faces = make_test_face_vector();
	Cube interesting_face_cube(faces);

	// Rotate the front face
	interesting_face_cube.rotate_back_face_cw();

	// Print face for manual checking
	interesting_face_cube.print_cube();
}

void RotateBackCCW_CalledNormally_CubeIsRotatedCorrectly() 
{
	// Create a mixed cube
	vector<Face> faces = make_test_face_vector();
	Cube interesting_face_cube(faces);

	// Rotate the front face
	interesting_face_cube.rotate_back_face_ccw();

	// Print face for manual checking
	interesting_face_cube.print_cube();
}

void RotateTopCW_CalledNormally_CubeIsRotatedCorrectly() 
{
	// Create a mixed cube
	vector<Face> faces = make_test_face_vector();
	Cube interesting_face_cube(faces);

	// Rotate the front face
	interesting_face_cube.rotate_top_face_cw();

	// Print face for manual checking
	interesting_face_cube.print_cube();
}

void RotateTopCCW_CalledNormally_CubeIsRotatedCorrectly() 
{
	// Create a mixed cube
	vector<Face> faces = make_test_face_vector();
	Cube interesting_face_cube(faces);

	// Rotate the front face
	interesting_face_cube.rotate_top_face_ccw();

	// Print face for manual checking
	interesting_face_cube.print_cube();
}

void RotateBottomCW_CalledNormally_CubeIsRotatedCorrectly() 
{
	// Create a mixed cube
	vector<Face> faces = make_test_face_vector();
	Cube interesting_face_cube(faces);

	// Rotate the front face
	interesting_face_cube.rotate_bottom_face_cw();

	// Print face for manual checking
	interesting_face_cube.print_cube();
}

void RotateBottomCCW_CalledNormally_CubeIsRotatedCorrectly() 
{
	// Create a mixed cube
	vector<Face> faces = make_test_face_vector();
	Cube interesting_face_cube(faces);

	// Rotate the front face
	interesting_face_cube.rotate_bottom_face_ccw();

	// Print face for manual checking
	interesting_face_cube.print_cube();
}

void RotateRightCW_CalledNormally_CubeIsRotatedCorrectly() 
{
	// Create a mixed cube
	vector<Face> faces = make_test_face_vector();
	Cube interesting_face_cube(faces);

	// Rotate the front face
	interesting_face_cube.rotate_right_face_cw();

	// Print face for manual checking
	interesting_face_cube.print_cube();
}

void RotateRightCCW_CalledNormally_CubeIsRotatedCorrectly() 
{
	// Create a mixed cube
	vector<Face> faces = make_test_face_vector();
	Cube interesting_face_cube(faces);

	// Rotate the front face
	interesting_face_cube.rotate_right_face_ccw();

	// Print face for manual checking
	interesting_face_cube.print_cube();
}

////////////////////////////////////////////////////////////////////////////////

// Helper Functions

// Make Test Color Matrix
// Arguments: n/a
// Returns: A color matrix which represents that needed to create a test face
// Purpose: Refactors making the same face every time for testing
vector< vector<int> > make_test_color_matrix() 
{
	vector< vector<int> > face_color_matrix;
	vector<int> temp_row1;
	vector<int> temp_row2;
	vector<int> temp_row3;

	temp_row1.push_back(WHITE);
	temp_row1.push_back(RED);
	temp_row1.push_back(WHITE);

	temp_row2.push_back(RED);
	temp_row2.push_back(RED);
	temp_row2.push_back(ORANGE);
	
	temp_row3.push_back(RED);
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(YELLOW);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	
	return face_color_matrix;
}

// Make Test Face Vector
// Arguments: n/a
// Returns: A color matrix which represents that needed to create a test face
// Purpose: Refactors making the same face every time for testing
vector<Face> make_test_face_vector() 
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

////////////////////////////////////////////////////////////////////////////////

