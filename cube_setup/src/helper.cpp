
////////////////////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: helper.cpp
// Purpose: Holds helper functions that may be used globally
//
////////////////////////////////////////////////////////////////////////////////

#include "helper.hpp"

using namespace std;

////////////////////////////////////////////////////////////////////////////////
//
// Swap Function (swaps two integers in place)
//
// first: pointer to first integer to be swapped
// second: pointer to second integer to be swapped
// returns: n/a
//
////////////////////////////////////////////////////////////////////////////////

void swap(int * first, int * second) 
{
	int temp = *first;
	*first = *second;
	*second = temp;
}

////////////////////////////////////////////////////////////////////////////////
//
// Reverse Function (reverses a given 3x3 vector)
//
// v: vector to be reversed
// returns: swapped vector
//
////////////////////////////////////////////////////////////////////////////////

vector<int> reverse(vector<int> v) 
{
	// if vector does not have 3 elements, throw invalid exception error
	if (v.size() != 3) 
	{
		throw invalid_argument("vector does not have 3 elements");
	}

	// otherwise, reverse the vector
	swap(&v[0], &v[2]);

	return v;
}

////////////////////////////////////////////////////////////////////////////////
//
// Print Square Function (prints out a single square of a Rubik's cube)
//
// color: color that should be printed
// returns: n/a
//
////////////////////////////////////////////////////////////////////////////////

void print_square(int color) 
{
	switch (color) 
	{
		case RED:
			cout << "\033[41;1m   \033[0m ";
			break;
		case ORANGE:
			cout << "\033[45;1m   \033[0m ";
			break;
		case BLUE:
			cout << "\033[44;1m   \033[0m ";
			break;
		case GREEN:
			cout << "\033[42;1m   \033[0m ";
			break;
		case WHITE:	
			cout << "\033[47;1m   \033[0m ";
			break;
		case YELLOW:
			cout << "\033[43;1m   \033[0m ";
			break;
		default:
			cout << "\033[40;1m   \033[0m ";
			break;
	}
}

////////////////////////////////////////////////////////////////////////////////

