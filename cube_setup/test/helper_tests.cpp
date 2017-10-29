
////////////////////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// File name: helper_test.cpp
// Purpose: Runs unit tests for helper functions
//
////////////////////////////////////////////////////////////////////////////////

#include <gtest/gtest.h>
#include <string>

#include "../src/helper.hpp"

using std::string;

int main(int argc, char **argv)
{
  	testing::InitGoogleTest(&argc, argv);
    	return RUN_ALL_TESTS();
}	

////////////////////////////////////////////////////////////////////////////////
//
// Swap Function Tests
// 	- properly called
//
////////////////////////////////////////////////////////////////////////////////

TEST(SwapTest, properlyCalled)
{
	// Arrange
	const int expected_a = 4242;
	const int expected_b = 42;
	int actual_a = expected_b;
	int actual_b = expected_a;
	
	//Act
	swap(&actual_a, &actual_b);
	
	//Assert
	ASSERT_EQ(expected_a, actual_a);
	ASSERT_EQ(expected_b, actual_b);
}


////////////////////////////////////////////////////////////////////////////////
//
// Reverse Function Tests
// 	- properly called
// 	- invalid v argument (throws exception)
//
////////////////////////////////////////////////////////////////////////////////

TEST(ReverseTest, properlyCalled)
{
	// Arrange
	const int v1 = 1;
	const int v2 = 5;
	const int v3 = 9;
	vector<int> expected_v;
	expected_v.push_back(v1);
	expected_v.push_back(v2);
	expected_v.push_back(v3);
	vector<int> actual_v;
	actual_v.push_back(v3);
	actual_v.push_back(v2);
	actual_v.push_back(v1);

	//Act
	actual_v = reverse(actual_v);
	
	//Assert
	ASSERT_EQ(expected_v, actual_v);
}

TEST(ReverseTest, invalidVArgument)
{
	// Arrange
	const string expected_error = "vector does not have 3 elements";
	string actual_error;
	vector<int> v;
	v.push_back(1);
	v.push_back(2);
	v.push_back(3);
	v.push_back(4);

	//Act
	try { v = reverse(v); }
	catch (invalid_argument const &e) { actual_error = e.what(); }
	
	//Assert
	ASSERT_EQ(expected_error, actual_error);
}

////////////////////////////////////////////////////////////////////////////////
//
// Print Square Function Tests
// 	- red works
// 	- orange works
// 	- blue works
// 	- green works
// 	- white works
// 	- yellow works
// 	-- default works
//
////////////////////////////////////////////////////////////////////////////////

TEST(PrintSquareTest, redWorks)
{
	// Arrange
	const string expected_output = "\033[41;1m   \033[0m ";
	const int expected_result = 0;
	std::stringstream actual_output;
	int color = RED;
	int actual_result;	

	//Act
	std::streambuf *sbuf = std::cout.rdbuf();
	std::cout.rdbuf(actual_output.rdbuf());
	print_square(color);
	actual_result = strcmp(expected_output.c_str(), actual_output.str().c_str());	
	std::cout.rdbuf(sbuf);
	
	//Assert
	ASSERT_EQ(expected_result, actual_result);
}

TEST(PrintSquareTest, orangeWorks)
{
	// Arrange
	const string expected_output = "\033[45;1m   \033[0m ";
	const int expected_result = 0;
	std::stringstream actual_output;
	int color = ORANGE;
	int actual_result;	

	//Act
	std::streambuf *sbuf = std::cout.rdbuf();
	std::cout.rdbuf(actual_output.rdbuf());
	print_square(color);
	actual_result = strcmp(expected_output.c_str(), actual_output.str().c_str());	
	std::cout.rdbuf(sbuf);
	
	//Assert
	ASSERT_EQ(expected_result, actual_result);
}

TEST(PrintSquareTest, blueWorks)
{
	// Arrange
	const string expected_output = "\033[44;1m   \033[0m ";
	const int expected_result = 0;
	std::stringstream actual_output;
	int color = BLUE;
	int actual_result;	

	//Act
	std::streambuf *sbuf = std::cout.rdbuf();
	std::cout.rdbuf(actual_output.rdbuf());
	print_square(color);
	actual_result = strcmp(expected_output.c_str(), actual_output.str().c_str());	
	std::cout.rdbuf(sbuf);
	
	//Assert
	ASSERT_EQ(expected_result, actual_result);
}

TEST(PrintSquareTest, greenWorks)
{
	// Arrange
	const string expected_output = "\033[42;1m   \033[0m ";
	const int expected_result = 0;
	std::stringstream actual_output;
	int color = GREEN;
	int actual_result;	

	//Act
	std::streambuf *sbuf = std::cout.rdbuf();
	std::cout.rdbuf(actual_output.rdbuf());
	print_square(color);
	actual_result = strcmp(expected_output.c_str(), actual_output.str().c_str());	
	std::cout.rdbuf(sbuf);
	
	//Assert
	ASSERT_EQ(expected_result, actual_result);
}

TEST(PrintSquareTest, whiteWorks)
{
	// Arrange
	const string expected_output = "\033[47;1m   \033[0m ";
	const int expected_result = 0;
	std::stringstream actual_output;
	int color = WHITE;
	int actual_result;	

	//Act
	std::streambuf *sbuf = std::cout.rdbuf();
	std::cout.rdbuf(actual_output.rdbuf());
	print_square(color);
	actual_result = strcmp(expected_output.c_str(), actual_output.str().c_str());	
	std::cout.rdbuf(sbuf);
	
	//Assert
	ASSERT_EQ(expected_result, actual_result);
}

TEST(PrintSquareTest, yellowWorks)
{
	// Arrange
	const string expected_output = "\033[43;1m   \033[0m ";
	const int expected_result = 0;
	std::stringstream actual_output;
	int color = YELLOW;
	int actual_result;	

	//Act
	std::streambuf *sbuf = std::cout.rdbuf();
	std::cout.rdbuf(actual_output.rdbuf());
	print_square(color);
	actual_result = strcmp(expected_output.c_str(), actual_output.str().c_str());	
	std::cout.rdbuf(sbuf);
	
	//Assert
	ASSERT_EQ(expected_result, actual_result);
}

TEST(PrintSquareTest, defaultWorks)
{
	// Arrange
	const string expected_output = "\033[40;1m   \033[0m ";
	const int expected_result = 0;
	std::stringstream actual_output;
	int color = -1;
	int actual_result;	

	//Act
	std::streambuf *sbuf = std::cout.rdbuf();
	std::cout.rdbuf(actual_output.rdbuf());
	print_square(color);
	actual_result = strcmp(expected_output.c_str(), actual_output.str().c_str());	
	std::cout.rdbuf(sbuf);
	
	//Assert
	ASSERT_EQ(expected_result, actual_result);
}

////////////////////////////////////////////////////////////////////////////////

