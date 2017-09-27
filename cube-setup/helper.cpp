
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

// Swap Function

void swap(int * first, int * second) {
	int temp = *first;
	*first = *second;
	*second = temp;
}

////////////////////////////////////////////////////////////////////////////////

// Print Square Function

void print_square(int color) {
	switch (color) {
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

// Append Vectors Function

vector<int> append_vectors(vector<int> v1, vector<int> v2) {

	vector<int> combined_vectors = v1;

	for (int i = 0; i < v2.size(); i++) {
		combined_vectors.push_back(v2[i]);
	}

	return combined_vectors;

}

////////////////////////////////////////////////////////////////////////////////

// Reverse 3x3 Vector

vector<int> reverse_vector(vector<int> v) {
	swap(&v[0], &v[2]);
	return v;
}

////////////////////////////////////////////////////////////////////////////////

