
////////////////////////////////////////////////////////////////////////////////
//
// Programmer: Victoria Albanese
// Filename: helper.hpp
// Purpose: Holds globally used information
//
////////////////////////////////////////////////////////////////////////////////

#ifndef HELPER_HPP
#define HELPER_HPP

#include <iostream>
#include <vector>

#define RED 0
#define ORANGE 1
#define BLUE 2
#define GREEN 3
#define WHITE 4
#define YELLOW 5

#define FRONT 0
#define BACK 1
#define TOP 2
#define BOTTOM 3
#define LEFT 4
#define RIGHT 5

using std::vector;

void swap(int * first, int * second);
void print_square(int color);
vector<int> append_vectors(vector<int> v1, vector<int> v2);
vector<int> reverse_vector(vector<int> v);

#endif // end of HELPER_HPP

////////////////////////////////////////////////////////////////////////////////

