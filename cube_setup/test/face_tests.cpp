
////////////////////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// File name: face_tests.cpp
// Purpose: Runs unit tests for face class member functions
//
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include <string>

#include "../src/helper.hpp"
#include "../src/face_class.hpp"

vector< vector<int> > make_color_matrix();
vector< vector<int> > make_color_matrix_cw();
vector< vector<int> > make_color_matrix_ccw();

using std::string;

int main(int argc, char **argv)
{
  	testing::InitGoogleTest(&argc, argv);
    	return RUN_ALL_TESTS();
}	

////////////////////////////////////////////////////////////////////////////////
//
// Default Constructor Tests
// 	- properly called
//
////////////////////////////////////////////////////////////////////////////////

TEST(DefaultConstructor, properlyCalled)
{
	// Arrange
	const int neg1 = -1;
	vector<int> expected_row;
	expected_row.push_back(neg1);
	expected_row.push_back(neg1);
	expected_row.push_back(neg1);
	vector< vector<int> > expected_face;
	expected_face.push_back(expected_row);
	expected_face.push_back(expected_row);
	expected_face.push_back(expected_row);
	
	//Act
	Face actual_face;
	
	//Assert
	ASSERT_EQ(expected_face, actual_face.get_face());
	ASSERT_EQ(expected_row, actual_face.get_row1());
	ASSERT_EQ(expected_row, actual_face.get_row2());
	ASSERT_EQ(expected_row, actual_face.get_row3());
}

////////////////////////////////////////////////////////////////////////////////
//
// One Argument Constructor Tests
// 	- properly called
// 	- color matrix invalid size (exception expected)
// 	- color matrix [0] invalid size (exception expected)
// 	- color matrix [1] invalid size (exception expected)
// 	- color matrix [2] invalid size (exception expected)
//
////////////////////////////////////////////////////////////////////////////////

TEST(OneArgumentConstructor, properlyCalled)
{
	// Arrange
	vector< vector<int> > expected_matrix = make_color_matrix();
	
	//Act
	Face actual_face(expected_matrix);
	
	//Assert
	ASSERT_EQ(expected_matrix, actual_face.get_face());
	ASSERT_EQ(expected_matrix[0], actual_face.get_row1());
	ASSERT_EQ(expected_matrix[1], actual_face.get_row2());
	ASSERT_EQ(expected_matrix[2], actual_face.get_row3());
}

TEST(OneArgumentConstructor, colorMatrixInvalidSize)
{
	// Arrange
	const string expected_error = "color matrix invalid size: must have three rows";
	vector< vector<int> > bad_matrix;
	string actual_error;
	
	//Act
	try { Face face(bad_matrix); }
	catch (invalid_argument const &e) { actual_error = e.what(); }
	
	//Assert
	ASSERT_EQ(expected_error, actual_error);
}

TEST(OneArgumentConstructor, colorMatrix0InvalidSize)
{
	// Arrange
	const string expected_error = "color_matrix row invalid size: must have three elements";
	vector< vector<int> > bad_matrix = make_color_matrix();
	bad_matrix[0].push_back(RED);
	string actual_error;
	
	//Act
	try { Face face(bad_matrix); }
	catch (invalid_argument const &e) { actual_error = e.what(); }
	
	//Assert
	ASSERT_EQ(expected_error, actual_error);
}

TEST(OneArgumentConstructor, colorMatrix1InvalidSize)
{
	// Arrange
	const string expected_error = "color_matrix row invalid size: must have three elements";
	vector< vector<int> > bad_matrix = make_color_matrix();
	bad_matrix[1].push_back(RED);
	string actual_error;
	
	//Act
	try { Face face(bad_matrix); }
	catch (invalid_argument const &e) { actual_error = e.what(); }
	
	//Assert
	ASSERT_EQ(expected_error, actual_error);
}

TEST(OneArgumentConstructor, colorMatrix2InvalidSize)
{
	// Arrange
	const string expected_error = "color_matrix row invalid size: must have three elements";
	vector< vector<int> > bad_matrix = make_color_matrix();
	bad_matrix[2].push_back(RED);
	string actual_error;
	
	//Act
	try { Face face(bad_matrix); }
	catch (invalid_argument const &e) { actual_error = e.what(); }
	
	//Assert
	ASSERT_EQ(expected_error, actual_error);
}

////////////////////////////////////////////////////////////////////////////////
//
// Equality Operator Tests
// 	- are equal
// 	- are not equal
//
////////////////////////////////////////////////////////////////////////////////

TEST(EqualityOperator, areEqual)
{
	// Arrange
	vector< vector<int> > matrix1 = make_color_matrix();
	vector< vector<int> > matrix2 = make_color_matrix();
	
	//Act
	Face face1(matrix1);
	Face face2(matrix2);

	//Assert
	ASSERT_TRUE(face1 == face2);
}

TEST(EqualityOperator, areNotEqual)
{
	// Arrange
	vector< vector<int> > matrix1 = make_color_matrix();
	vector< vector<int> > matrix2 = make_color_matrix_cw();
	
	//Act
	Face face1(matrix1);
	Face face2(matrix2);

	//Assert
	ASSERT_FALSE(face1 == face2);
}

////////////////////////////////////////////////////////////////////////////////
//
// Inequality Operator Tests
// 	- are equal
// 	- are not equal
//
////////////////////////////////////////////////////////////////////////////////

TEST(InequalityOperator, areEqual)
{
	// Arrange
	vector< vector<int> > matrix1 = make_color_matrix();
	vector< vector<int> > matrix2 = make_color_matrix();
	
	//Act
	Face face1(matrix1);
	Face face2(matrix2);

	//Assert
	ASSERT_FALSE(face1 != face2);
}

TEST(InequalityOperator, areNotEqual)
{
	// Arrange
	vector< vector<int> > matrix1 = make_color_matrix();
	vector< vector<int> > matrix2 = make_color_matrix_cw();
	
	//Act
	Face face1(matrix1);
	Face face2(matrix2);

	//Assert
	ASSERT_TRUE(face1 != face2);
}

////////////////////////////////////////////////////////////////////////////////
//
// Set Face Function Tests
// 	- properly called
// 	- color matrix invalid size (exception expected)
// 	- color matrix [0] invalid size (exception expected)
// 	- color matrix [1] invalid size (exception expected)
// 	- color matrix [2] invalid size (exception expected)
//
////////////////////////////////////////////////////////////////////////////////

TEST(SetFace, properlyCalled)
{
	// Arrange
	vector< vector<int> > old_matrix = make_color_matrix();
	vector< vector<int> > new_matrix = make_color_matrix_cw();
	Face actual_face(old_matrix);
	Face expected_face(new_matrix);
	
	//Act
	actual_face.set_face(new_matrix);
	
	//Assert
	ASSERT_EQ(expected_face, actual_face);
}

TEST(SetFace, colorMatrixInvalidSize)
{
	// Arrange
	const string expected_error = "color matrix invalid size: must have three rows";
	vector< vector<int> > bad_matrix;
	Face face(make_color_matrix());
	string actual_error;
	
	//Act
	try { face.set_face(bad_matrix); }
	catch (invalid_argument const &e) { actual_error = e.what(); }
	
	//Assert
	ASSERT_EQ(expected_error, actual_error);
}

TEST(SetFace, colorMatrix0InvalidSize)
{
	// Arrange
	const string expected_error = "color_matrix row invalid size: must have three elements";
	vector< vector<int> > bad_matrix = make_color_matrix();
	bad_matrix[0].push_back(RED);
	Face face(make_color_matrix());
	string actual_error;
	
	//Act
	try { face.set_face(bad_matrix); }
	catch (invalid_argument const &e) { actual_error = e.what(); }
	
	//Assert
	ASSERT_EQ(expected_error, actual_error);
}

TEST(SetFace, colorMatrix1InvalidSize)
{
	// Arrange
	const string expected_error = "color_matrix row invalid size: must have three elements";
	vector< vector<int> > bad_matrix = make_color_matrix();
	bad_matrix[1].push_back(RED);
	Face face(make_color_matrix());
	string actual_error;
	
	//Act
	try { face.set_face(bad_matrix); }
	catch (invalid_argument const &e) { actual_error = e.what(); }
	
	//Assert
	ASSERT_EQ(expected_error, actual_error);
}

TEST(SetFace, colorMatrix2InvalidSize)
{
	// Arrange
	const string expected_error = "color_matrix row invalid size: must have three elements";
	vector< vector<int> > bad_matrix = make_color_matrix();
	bad_matrix[2].push_back(RED);
	Face face(make_color_matrix());
	string actual_error;
	
	//Act
	try { face.set_face(bad_matrix); }
	catch (invalid_argument const &e) { actual_error = e.what(); }
	
	//Assert
	ASSERT_EQ(expected_error, actual_error);
}

////////////////////////////////////////////////////////////////////////////////
//
// Set Row1 Function Tests
// 	- properly called
// 	- new row invalid size (exception expected)
//
////////////////////////////////////////////////////////////////////////////////

TEST(SetRow1, properlyCalled)
{
	// Arrange
	vector< vector<int> > new_matrix;
	vector< vector<int> > old_matrix = make_color_matrix();
	vector<int> new_vector;
	new_vector.push_back(WHITE);
	new_vector.push_back(WHITE);
	new_vector.push_back(WHITE);
	new_matrix.push_back(new_vector);
	new_matrix.push_back(old_matrix[1]);
	new_matrix.push_back(old_matrix[2]);
	Face actual_face(old_matrix);
	Face expected_face(new_matrix);

	//Act
	actual_face.set_row1(new_vector);
	
	//Assert
	ASSERT_EQ(expected_face, actual_face);
}

TEST(SetRow1, newRowInvalidSize)
{
	// Arrange
	const string expected_error = "new row invalid size: must have three elements";
	Face actual_face(make_color_matrix());
	vector<int> bad_row;
	string actual_error;
	
	//Act
	try { actual_face.set_row1(bad_row); }
	catch (invalid_argument const &e) { actual_error = e.what(); }
	
	//Assert
	ASSERT_EQ(expected_error, actual_error);
}

////////////////////////////////////////////////////////////////////////////////
//
// Set Row2 Function Tests
// 	- properly called
// 	- new row invalid size (exception expected)
//
////////////////////////////////////////////////////////////////////////////////

TEST(SetRow2, properlyCalled)
{
	// Arrange
	vector< vector<int> > new_matrix;
	vector< vector<int> > old_matrix = make_color_matrix();
	vector<int> new_vector;
	new_vector.push_back(WHITE);
	new_vector.push_back(WHITE);
	new_vector.push_back(WHITE);
	new_matrix.push_back(old_matrix[0]);
	new_matrix.push_back(new_vector);
	new_matrix.push_back(old_matrix[2]);
	Face actual_face(old_matrix);
	Face expected_face(new_matrix);

	//Act
	actual_face.set_row2(new_vector);
	
	//Assert
	ASSERT_EQ(expected_face, actual_face);
}

TEST(SetRow2, newRowInvalidSize)
{
	// Arrange
	const string expected_error = "new row invalid size: must have three elements";
	Face actual_face(make_color_matrix());
	vector<int> bad_row;
	string actual_error;
	
	//Act
	try { actual_face.set_row2(bad_row); }
	catch (invalid_argument const &e) { actual_error = e.what(); }
	
	//Assert
	ASSERT_EQ(expected_error, actual_error);
}

////////////////////////////////////////////////////////////////////////////////
//
// Set Row3 Function Tests
// 	- properly called
// 	- new row invalid size (exception expected)
//
////////////////////////////////////////////////////////////////////////////////

TEST(SetRow3, properlyCalled)
{
	// Arrange
	vector< vector<int> > new_matrix;
	vector< vector<int> > old_matrix = make_color_matrix();
	vector<int> new_vector;
	new_vector.push_back(WHITE);
	new_vector.push_back(WHITE);
	new_vector.push_back(WHITE);
	new_matrix.push_back(old_matrix[0]);
	new_matrix.push_back(old_matrix[1]);
	new_matrix.push_back(new_vector);
	Face actual_face(old_matrix);
	Face expected_face(new_matrix);
	
	//Act
	actual_face.set_row3(new_vector);
	
	//Assert
	ASSERT_EQ(expected_face, actual_face);
}

TEST(SetRow3, newRowInvalidSize)
{
	// Arrange
	const string expected_error = "new row invalid size: must have three elements";
	Face actual_face(make_color_matrix());
	vector<int> bad_row;
	string actual_error;
	
	//Act
	try { actual_face.set_row3(bad_row); }
	catch (invalid_argument const &e) { actual_error = e.what(); }
	
	//Assert
	ASSERT_EQ(expected_error, actual_error);
}

////////////////////////////////////////////////////////////////////////////////
//
// Rotate CW Function Tests
// 	- properly called
//
////////////////////////////////////////////////////////////////////////////////

TEST(RotateCW, properlyCalled)
{
	// Arrange
	Face actual_face(make_color_matrix());
	Face expected_face(make_color_matrix_cw());
	
	//Act
	actual_face.rotate_cw();
	
	//Assert
	ASSERT_EQ(expected_face, actual_face);
}

////////////////////////////////////////////////////////////////////////////////
//
// Rotate CCW Function Tests
// 	- properly called
//
////////////////////////////////////////////////////////////////////////////////

TEST(RotateCCW, properlyCalled)
{
	// Arrange
	Face actual_face(make_color_matrix());
	Face expected_face(make_color_matrix_ccw());
	
	//Act
	actual_face.rotate_ccw();
	
	//Assert
	ASSERT_EQ(expected_face, actual_face);
}

////////////////////////////////////////////////////////////////////////////////

// Helper Functions

// Make Test Color Matrix
// Arguments: n/a
// Returns: A color matrix which represents that needed to create a test face
// Purpose: Refactors making the same face every time for testing
vector< vector<int> > make_color_matrix() 
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

// Make Test Color Matrix CW
// Arguments: n/a
// Returns: A color matrix which represents that needed to create a test face
// Purpose: Refactors making the same face every time for testing
vector< vector<int> > make_color_matrix_cw() 
{
	vector< vector<int> > face_color_matrix;
	vector<int> temp_row1;
	vector<int> temp_row2;
	vector<int> temp_row3;

	temp_row1.push_back(RED);
	temp_row1.push_back(RED);
	temp_row1.push_back(WHITE);

	temp_row2.push_back(YELLOW);
	temp_row2.push_back(RED);
	temp_row2.push_back(RED);
	
	temp_row3.push_back(YELLOW);
	temp_row3.push_back(ORANGE);
	temp_row3.push_back(WHITE);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	
	return face_color_matrix;
}

// Make Test Color Matrix CCW
// Arguments: n/a
// Returns: A color matrix which represents that needed to create a test face
// Purpose: Refactors making the same face every time for testing
vector< vector<int> > make_color_matrix_ccw() 
{
	vector< vector<int> > face_color_matrix;
	vector<int> temp_row1;
	vector<int> temp_row2;
	vector<int> temp_row3;

	temp_row1.push_back(WHITE);
	temp_row1.push_back(ORANGE);
	temp_row1.push_back(YELLOW);

	temp_row2.push_back(RED);
	temp_row2.push_back(RED);
	temp_row2.push_back(YELLOW);
	
	temp_row3.push_back(WHITE);
	temp_row3.push_back(RED);
	temp_row3.push_back(RED);

	face_color_matrix.push_back(temp_row1);
	face_color_matrix.push_back(temp_row2);
	face_color_matrix.push_back(temp_row3);
	
	return face_color_matrix;
}

////////////////////////////////////////////////////////////////////////////////

