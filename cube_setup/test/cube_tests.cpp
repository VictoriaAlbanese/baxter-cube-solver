
////////////////////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// File name: cube_tests.cpp
// Purpose: Runs unit tests for cube class member functions
//
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include <string>

#include "../src/helper.hpp"
#include "../src/face_class.hpp"
#include "../src/cube_class.hpp"

#include "solved_cube_builders.cpp"
#include "test_cube1_builders.cpp"
#include "test_cube2_builders.cpp"

#define FRONT 0
#define BACK 1
#define TOP 2
#define BOTTOM 3
#define LEFT 4
#define RIGHT 5

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
	vector<Face> expected_faces;
	Face new_face;
	expected_faces.push_back(new_face);
	expected_faces.push_back(new_face);
	expected_faces.push_back(new_face);
	expected_faces.push_back(new_face);
	expected_faces.push_back(new_face);
	expected_faces.push_back(new_face);

	//Act
	Cube actual_cube;
	
	//Assert
	ASSERT_EQ(expected_faces[FRONT], actual_cube.get_front_face());
	ASSERT_EQ(expected_faces[BACK], actual_cube.get_back_face());
	ASSERT_EQ(expected_faces[TOP], actual_cube.get_top_face());
	ASSERT_EQ(expected_faces[BOTTOM], actual_cube.get_bottom_face());
	ASSERT_EQ(expected_faces[LEFT], actual_cube.get_left_face());
	ASSERT_EQ(expected_faces[RIGHT], actual_cube.get_right_face());
	ASSERT_EQ(expected_faces[TOP], *actual_cube.get_front_face().get_adjacent_top());
	ASSERT_EQ(expected_faces[BOTTOM], *actual_cube.get_front_face().get_adjacent_bottom());
	ASSERT_EQ(expected_faces[LEFT], *actual_cube.get_front_face().get_adjacent_left());
	ASSERT_EQ(expected_faces[RIGHT], *actual_cube.get_front_face().get_adjacent_right());
	ASSERT_EQ(expected_faces[TOP], *actual_cube.get_back_face().get_adjacent_top());
	ASSERT_EQ(expected_faces[BOTTOM], *actual_cube.get_back_face().get_adjacent_bottom());
	ASSERT_EQ(expected_faces[RIGHT], *actual_cube.get_back_face().get_adjacent_left());
	ASSERT_EQ(expected_faces[LEFT], *actual_cube.get_back_face().get_adjacent_right());
	ASSERT_EQ(expected_faces[BACK], *actual_cube.get_top_face().get_adjacent_top());
	ASSERT_EQ(expected_faces[FRONT], *actual_cube.get_top_face().get_adjacent_bottom());
	ASSERT_EQ(expected_faces[LEFT], *actual_cube.get_top_face().get_adjacent_left());
	ASSERT_EQ(expected_faces[RIGHT], *actual_cube.get_top_face().get_adjacent_right());
	ASSERT_EQ(expected_faces[FRONT], *actual_cube.get_bottom_face().get_adjacent_top());
	ASSERT_EQ(expected_faces[BACK], *actual_cube.get_bottom_face().get_adjacent_bottom());
	ASSERT_EQ(expected_faces[LEFT], *actual_cube.get_bottom_face().get_adjacent_left());
	ASSERT_EQ(expected_faces[RIGHT], *actual_cube.get_bottom_face().get_adjacent_right());
	ASSERT_EQ(expected_faces[TOP], *actual_cube.get_left_face().get_adjacent_top());
	ASSERT_EQ(expected_faces[BOTTOM], *actual_cube.get_left_face().get_adjacent_bottom());
	ASSERT_EQ(expected_faces[BACK], *actual_cube.get_left_face().get_adjacent_left());
	ASSERT_EQ(expected_faces[FRONT], *actual_cube.get_left_face().get_adjacent_right());
	ASSERT_EQ(expected_faces[TOP], *actual_cube.get_right_face().get_adjacent_top());
	ASSERT_EQ(expected_faces[BOTTOM], *actual_cube.get_right_face().get_adjacent_bottom());
	ASSERT_EQ(expected_faces[FRONT], *actual_cube.get_right_face().get_adjacent_left());
	ASSERT_EQ(expected_faces[BACK], *actual_cube.get_right_face().get_adjacent_right());
}

////////////////////////////////////////////////////////////////////////////////
//
// One Argument Constructor Tests
// 	- properly called
// 	- faces invalid size (exception expected)
//
////////////////////////////////////////////////////////////////////////////////

TEST(OneArgumentConstructor, properlyCalled)
{
	// Arrange
	vector<Face> expected_faces = make_solved_face_vector();

	//Act
	Cube actual_cube(expected_faces);
	
	//Assert
	ASSERT_EQ(expected_faces[0], actual_cube.get_front_face());
	ASSERT_EQ(expected_faces[1], actual_cube.get_back_face());
	ASSERT_EQ(expected_faces[2], actual_cube.get_top_face());
	ASSERT_EQ(expected_faces[3], actual_cube.get_bottom_face());
	ASSERT_EQ(expected_faces[4], actual_cube.get_left_face());
	ASSERT_EQ(expected_faces[5], actual_cube.get_right_face());
	ASSERT_EQ(expected_faces[TOP], *actual_cube.get_front_face().get_adjacent_top());
	ASSERT_EQ(expected_faces[BOTTOM], *actual_cube.get_front_face().get_adjacent_bottom());
	ASSERT_EQ(expected_faces[LEFT], *actual_cube.get_front_face().get_adjacent_left());
	ASSERT_EQ(expected_faces[RIGHT], *actual_cube.get_front_face().get_adjacent_right());
	ASSERT_EQ(expected_faces[TOP], *actual_cube.get_back_face().get_adjacent_top());
	ASSERT_EQ(expected_faces[BOTTOM], *actual_cube.get_back_face().get_adjacent_bottom());
	ASSERT_EQ(expected_faces[RIGHT], *actual_cube.get_back_face().get_adjacent_left());
	ASSERT_EQ(expected_faces[LEFT], *actual_cube.get_back_face().get_adjacent_right());
	ASSERT_EQ(expected_faces[BACK], *actual_cube.get_top_face().get_adjacent_top());
	ASSERT_EQ(expected_faces[FRONT], *actual_cube.get_top_face().get_adjacent_bottom());
	ASSERT_EQ(expected_faces[LEFT], *actual_cube.get_top_face().get_adjacent_left());
	ASSERT_EQ(expected_faces[RIGHT], *actual_cube.get_top_face().get_adjacent_right());
	ASSERT_EQ(expected_faces[FRONT], *actual_cube.get_bottom_face().get_adjacent_top());
	ASSERT_EQ(expected_faces[BACK], *actual_cube.get_bottom_face().get_adjacent_bottom());
	ASSERT_EQ(expected_faces[LEFT], *actual_cube.get_bottom_face().get_adjacent_left());
	ASSERT_EQ(expected_faces[RIGHT], *actual_cube.get_bottom_face().get_adjacent_right());
	ASSERT_EQ(expected_faces[TOP], *actual_cube.get_left_face().get_adjacent_top());
	ASSERT_EQ(expected_faces[BOTTOM], *actual_cube.get_left_face().get_adjacent_bottom());
	ASSERT_EQ(expected_faces[BACK], *actual_cube.get_left_face().get_adjacent_left());
	ASSERT_EQ(expected_faces[FRONT], *actual_cube.get_left_face().get_adjacent_right());
	ASSERT_EQ(expected_faces[TOP], *actual_cube.get_right_face().get_adjacent_top());
	ASSERT_EQ(expected_faces[BOTTOM], *actual_cube.get_right_face().get_adjacent_bottom());
	ASSERT_EQ(expected_faces[FRONT], *actual_cube.get_right_face().get_adjacent_left());
	ASSERT_EQ(expected_faces[BACK], *actual_cube.get_right_face().get_adjacent_right());

}

TEST(OneArgumentConstructor, facesInvalidSize)
{
	// Arrange
	const string expected_error = "Invalid number of faces: expected 6";
	vector<Face> bad_faces;
	string actual_error;

	//Act
	try { Cube cube(bad_faces); }
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
	vector<Face> faces1 = make_solved_face_vector();
	vector<Face> faces2 = make_solved_face_vector();
	
	//Act
	Cube cube1(faces1);
	Cube cube2(faces2);

	//Assert
	ASSERT_TRUE(cube1 == cube2);
}

TEST(EqualityOperator, areNotEqual)
{
	// Arrange
	vector<Face> faces1 = make_solved_face_vector();
	vector<Face> faces2 = make_solved_face_vector_front_cw();
	
	//Act
	Cube cube1(faces1);
	Cube cube2(faces2);

	//Assert
	ASSERT_FALSE(cube1 == cube2);
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
	vector<Face> faces1 = make_solved_face_vector();
	vector<Face> faces2 = make_solved_face_vector();
	
	//Act
	Cube cube1(faces1);
	Cube cube2(faces2);

	//Assert
	ASSERT_FALSE(cube1 != cube2);
}

TEST(InequalityOperator, areNotEqual)
{
	// Arrange
	vector<Face> faces1 = make_solved_face_vector();
	vector<Face> faces2 = make_solved_face_vector_front_cw();
	
	//Act
	Cube cube1(faces1);
	Cube cube2(faces2);

	//Assert
	ASSERT_TRUE(cube1 != cube2);
}

////////////////////////////////////////////////////////////////////////////////
//
// Rotate Front Face Tests
// 	- clockwise works, solved cube
// 	- clockwise works, random test cube 1
// 	- clockwise works, random test cube 2
// 	- counterclockwise works, solved cube
// 	- counterclockwise works, random test cube 1
// 	- counterclockwise works, random test cube 2
// 	- invalid direction (exception expected)
//
////////////////////////////////////////////////////////////////////////////////

TEST(RotateFrontFace, clockwiseWorksSolvedCube)
{
	// Arrange
	Cube expected_cube(make_solved_face_vector_front_cw());
	Cube actual_cube(make_solved_face_vector());

	//Act
	actual_cube.rotate_front_face(CW);
	
	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateFrontFace, clockwiseWorksTestCube1)
{
	// Arrange
	Cube expected_cube(make_test_face_vector1_front_cw());
	Cube actual_cube(make_test_face_vector1());

	//Act
	actual_cube.rotate_front_face(CW);
	
	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateFrontFace, clockwiseWorksTestCube2)
{
	// Arrange
	Cube expected_cube(make_test_face_vector2_front_cw());
	Cube actual_cube(make_test_face_vector2());

	//Act
	actual_cube.rotate_front_face(CW);
	
	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateFrontFace, counterClockwiseWorksSolvedCube)
{
	// Arrange
	Cube expected_cube(make_solved_face_vector_front_ccw());
	Cube actual_cube(make_solved_face_vector());

	//Act
	actual_cube.rotate_front_face(CCW);

	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateFrontFace, counterClockwiseWorksTestCube1)
{
	// Arrange
	Cube expected_cube(make_test_face_vector1_front_ccw());
	Cube actual_cube(make_test_face_vector1());

	//Act
	actual_cube.rotate_front_face(CCW);

	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateFrontFace, counterClockwiseWorksTestCube2)
{
	// Arrange
	Cube expected_cube(make_test_face_vector2_front_ccw());
	Cube actual_cube(make_test_face_vector2());

	//Act
	actual_cube.rotate_front_face(CCW);

	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateFrontFace, invalidDirection)
{
	// Arrange
	const string expected_error = "Invalid direction: expected CW or CCW";
	Cube cube(make_solved_face_vector());
	int bad_direction = -1;
	string actual_error;

	//Act
	try { cube.rotate_front_face(bad_direction); }
	catch (invalid_argument const &e) { actual_error = e.what(); }
	
	//Assert
	ASSERT_EQ(expected_error, actual_error);
}

////////////////////////////////////////////////////////////////////////////////
//
// Rotate Back Face Tests
// 	- clockwise works, solved cube
// 	- clockwise works, random test cube 1
// 	- clockwise works, random test cube 2
// 	- counterclockwise works, solved cube
// 	- counterclockwise works, random test cube 1
// 	- counterclockwise works, random test cube 2
// 	- invalid direction (exception expected)
//
////////////////////////////////////////////////////////////////////////////////

TEST(RotateBackFace, clockwiseWorksSolvedCube)
{
	// Arrange
	Cube expected_cube(make_solved_face_vector_back_cw());
	Cube actual_cube(make_solved_face_vector());

	//Act
	actual_cube.rotate_back_face(CW);
	
	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateBackFace, clockwiseWorksTestCube1)
{
	// Arrange
	Cube expected_cube(make_test_face_vector1_back_cw());
	Cube actual_cube(make_test_face_vector1());

	//Act
	actual_cube.rotate_back_face(CW);
	
	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateBackFace, clockwiseWorksTestCube2)
{
	// Arrange
	Cube expected_cube(make_test_face_vector2_back_cw());
	Cube actual_cube(make_test_face_vector2());

	//Act
	actual_cube.rotate_back_face(CW);
	
	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateBackFace, counterClockwiseWorksSolvedCube)
{
	// Arrange
	Cube expected_cube(make_solved_face_vector_back_ccw());
	Cube actual_cube(make_solved_face_vector());

	//Act
	actual_cube.rotate_back_face(CCW);

	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateBackFace, counterClockwiseWorksTestCube1)
{
	// Arrange
	Cube expected_cube(make_test_face_vector1_back_ccw());
	Cube actual_cube(make_test_face_vector1());

	//Act
	actual_cube.rotate_back_face(CCW);

	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateBackFace, counterClockwiseWorksTestCube2)
{
	// Arrange
	Cube expected_cube(make_test_face_vector2_back_ccw());
	Cube actual_cube(make_test_face_vector2());

	//Act
	actual_cube.rotate_back_face(CCW);

	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateBackFace, invalidDirection)
{
	// Arrange
	const string expected_error = "Invalid direction: expected CW or CCW";
	Cube cube(make_solved_face_vector());
	int bad_direction = -1;
	string actual_error;

	//Act
	try { cube.rotate_back_face(bad_direction); }
	catch (invalid_argument const &e) { actual_error = e.what(); }
	
	//Assert
	ASSERT_EQ(expected_error, actual_error);
}

////////////////////////////////////////////////////////////////////////////////
//
// Rotate Top Face Tests
// 	- clockwise works, solved cube
// 	- clockwise works, random test cube 1
// 	- clockwise works, random test cube 2
// 	- counterclockwise works, solved cube
// 	- counterclockwise works, random test cube 1
// 	- counterclockwise works, random test cube 2
// 	- invalid direction (exception expected)
//
////////////////////////////////////////////////////////////////////////////////

TEST(RotateTopFace, clockwiseWorksSolvedCube)
{
	// Arrange
	Cube expected_cube(make_solved_face_vector_top_cw());
	Cube actual_cube(make_solved_face_vector());

	//Act
	actual_cube.rotate_top_face(CW);
	
	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateTopFace, clockwiseWorksTestCube1)
{
	// Arrange
	Cube expected_cube(make_test_face_vector1_top_cw());
	Cube actual_cube(make_test_face_vector1());

	//Act
	actual_cube.rotate_top_face(CW);
	
	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateTopFace, clockwiseWorksTestCube2)
{
	// Arrange
	Cube expected_cube(make_test_face_vector2_top_cw());
	Cube actual_cube(make_test_face_vector2());

	//Act
	actual_cube.rotate_top_face(CW);
	
	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateTopFace, counterClockwiseWorksSolvedCube)
{
	// Arrange
	Cube expected_cube(make_solved_face_vector_top_ccw());
	Cube actual_cube(make_solved_face_vector());

	//Act
	actual_cube.rotate_top_face(CCW);

	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateTopFace, counterClockwiseWorksTestCube1)
{
	// Arrange
	Cube expected_cube(make_test_face_vector1_top_ccw());
	Cube actual_cube(make_test_face_vector1());

	//Act
	actual_cube.rotate_top_face(CCW);

	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateTopFace, counterClockwiseWorksTestCube2)
{
	// Arrange
	Cube expected_cube(make_test_face_vector2_top_ccw());
	Cube actual_cube(make_test_face_vector2());

	//Act
	actual_cube.rotate_top_face(CCW);

	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateTopFace, invalidDirection)
{
	// Arrange
	const string expected_error = "Invalid direction: expected CW or CCW";
	Cube cube(make_solved_face_vector());
	int bad_direction = -1;
	string actual_error;

	//Act
	try { cube.rotate_top_face(bad_direction); }
	catch (invalid_argument const &e) { actual_error = e.what(); }
	
	//Assert
	ASSERT_EQ(expected_error, actual_error);
}

////////////////////////////////////////////////////////////////////////////////
//
// Rotate Bottom Face Tests
// 	- clockwise works, solved cube
// 	- clockwise works, random test cube 1
// 	- clockwise works, random test cube 2
// 	- counterclockwise works, solved cube
// 	- counterclockwise works, random test cube 1
// 	- counterclockwise works, random test cube 2
// 	- invalid direction (exception expected)
//
////////////////////////////////////////////////////////////////////////////////

TEST(RotateBottomFace, clockwiseWorksSolvedCube)
{
	// Arrange
	Cube expected_cube(make_solved_face_vector_bottom_cw());
	Cube actual_cube(make_solved_face_vector());

	//Act
	actual_cube.rotate_bottom_face(CW);
	
	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateBottomFace, clockwiseWorksTestCube1)
{
	// Arrange
	Cube expected_cube(make_test_face_vector1_bottom_cw());
	Cube actual_cube(make_test_face_vector1());

	//Act
	actual_cube.rotate_bottom_face(CW);
	
	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateBottomFace, clockwiseWorksTestCube2)
{
	// Arrange
	Cube expected_cube(make_test_face_vector2_bottom_cw());
	Cube actual_cube(make_test_face_vector2());

	//Act
	actual_cube.rotate_bottom_face(CW);
	
	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateBottomFace, counterClockwiseWorksSolvedCube)
{
	// Arrange
	Cube expected_cube(make_solved_face_vector_bottom_ccw());
	Cube actual_cube(make_solved_face_vector());

	//Act
	actual_cube.rotate_bottom_face(CCW);

	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateBottomFace, counterClockwiseWorksTestCube1)
{
	// Arrange
	Cube expected_cube(make_test_face_vector1_bottom_ccw());
	Cube actual_cube(make_test_face_vector1());

	//Act
	actual_cube.rotate_bottom_face(CCW);

	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateBottomFace, counterClockwiseWorksTestCube2)
{
	// Arrange
	Cube expected_cube(make_test_face_vector2_bottom_ccw());
	Cube actual_cube(make_test_face_vector2());

	//Act
	actual_cube.rotate_bottom_face(CCW);

	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateBottomFace, invalidDirection)
{
	// Arrange
	const string expected_error = "Invalid direction: expected CW or CCW";
	Cube cube(make_solved_face_vector());
	int bad_direction = -1;
	string actual_error;

	//Act
	try { cube.rotate_bottom_face(bad_direction); }
	catch (invalid_argument const &e) { actual_error = e.what(); }
	
	//Assert
	ASSERT_EQ(expected_error, actual_error);
}

////////////////////////////////////////////////////////////////////////////////
//
// Rotate Left Face Tests
// 	- clockwise works, solved cube
// 	- clockwise works, random test cube 1
// 	- clockwise works, random test cube 2
// 	- counterclockwise works, solved cube
// 	- counterclockwise works, random test cube 1
// 	- counterclockwise works, random test cube 2
// 	- invalid direction (exception expected)
//
////////////////////////////////////////////////////////////////////////////////

TEST(RotateLeftFace, clockwiseWorksSolvedCube)
{
	// Arrange
	Cube expected_cube(make_solved_face_vector_left_cw());
	Cube actual_cube(make_solved_face_vector());

	//Act
	actual_cube.rotate_left_face(CW);
	
	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateLeftFace, clockwiseWorksTestCube1)
{
	// Arrange
	Cube expected_cube(make_test_face_vector1_left_cw());
	Cube actual_cube(make_test_face_vector1());

	//Act
	actual_cube.rotate_left_face(CW);
	
	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateLeftFace, clockwiseWorksTestCube2)
{
	// Arrange
	Cube expected_cube(make_test_face_vector2_left_cw());
	Cube actual_cube(make_test_face_vector2());

	//Act
	actual_cube.rotate_left_face(CW);
	
	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateLeftFace, counterClockwiseWorksSolvedCube)
{
	// Arrange
	Cube expected_cube(make_solved_face_vector_left_ccw());
	Cube actual_cube(make_solved_face_vector());

	//Act
	actual_cube.rotate_left_face(CCW);

	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateLeftFace, counterClockwiseWorksTestCube1)
{
	// Arrange
	Cube expected_cube(make_test_face_vector1_left_ccw());
	Cube actual_cube(make_test_face_vector1());

	//Act
	actual_cube.rotate_left_face(CCW);

	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateLeftFace, counterClockwiseWorksTestCube2)
{
	// Arrange
	Cube expected_cube(make_test_face_vector2_left_ccw());
	Cube actual_cube(make_test_face_vector2());

	//Act
	actual_cube.rotate_left_face(CCW);

	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateLeftFace, invalidDirection)
{
	// Arrange
	const string expected_error = "Invalid direction: expected CW or CCW";
	Cube cube(make_solved_face_vector());
	int bad_direction = -1;
	string actual_error;

	//Act
	try { cube.rotate_left_face(bad_direction); }
	catch (invalid_argument const &e) { actual_error = e.what(); }
	
	//Assert
	ASSERT_EQ(expected_error, actual_error);
}

////////////////////////////////////////////////////////////////////////////////
//
// Rotate Right Face Tests
// 	- clockwise works, solved cube
// 	- clockwise works, random test cube 1
// 	- clockwise works, random test cube 2
// 	- counterclockwise works, solved cube
// 	- counterclockwise works, random test cube 1
// 	- counterclockwise works, random test cube 2
// 	- invalid direction (exception expected)
//
////////////////////////////////////////////////////////////////////////////////

TEST(RotateRightFace, clockwiseWorksSolvedCube)
{
	// Arrange
	Cube expected_cube(make_solved_face_vector_right_cw());
	Cube actual_cube(make_solved_face_vector());

	//Act
	actual_cube.rotate_right_face(CW);
	
	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateRightFace, clockwiseWorksTestCube1)
{
	// Arrange
	Cube expected_cube(make_test_face_vector1_right_cw());
	Cube actual_cube(make_test_face_vector1());

	//Act
	actual_cube.rotate_right_face(CW);
	
	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateRightFace, clockwiseWorksTestCube2)
{
	// Arrange
	Cube expected_cube(make_test_face_vector2_right_cw());
	Cube actual_cube(make_test_face_vector2());

	//Act
	actual_cube.rotate_right_face(CW);
	
	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateRightFace, counterClockwiseWorksSolvedCube)
{
	// Arrange
	Cube expected_cube(make_solved_face_vector_right_ccw());
	Cube actual_cube(make_solved_face_vector());

	//Act
	actual_cube.rotate_right_face(CCW);

	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateRightFace, counterClockwiseWorksTestCube1)
{
	// Arrange
	Cube expected_cube(make_test_face_vector1_right_ccw());
	Cube actual_cube(make_test_face_vector1());

	//Act
	actual_cube.rotate_right_face(CCW);

	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateRightFace, counterClockwiseWorksTestCube2)
{
	// Arrange
	Cube expected_cube(make_test_face_vector2_right_ccw());
	Cube actual_cube(make_test_face_vector2());

	//Act
	actual_cube.rotate_right_face(CCW);

	//Assert
	ASSERT_EQ(expected_cube, actual_cube);
}

TEST(RotateRightFace, invalidDirection)
{
	// Arrange
	const string expected_error = "Invalid direction: expected CW or CCW";
	Cube cube(make_solved_face_vector());
	int bad_direction = -1;
	string actual_error;

	//Act
	try { cube.rotate_right_face(bad_direction); }
	catch (invalid_argument const &e) { actual_error = e.what(); }
	
	//Assert
	ASSERT_EQ(expected_error, actual_error);
}

////////////////////////////////////////////////////////////////////////////////

