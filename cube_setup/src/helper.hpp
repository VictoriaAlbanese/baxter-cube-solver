
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
#include <stdexcept>
#include <vector>

#define RED 0
#define ORANGE 1
#define BLUE 2
#define GREEN 3
#define WHITE 4
#define YELLOW 5

using std::invalid_argument;
using std::vector;

void swap(int * first, int * second);
vector<int> reverse(vector<int> v);
void print_square(int color);

#endif // end of HELPER_HPP

////////////////////////////////////////////////////////////////////////////////

