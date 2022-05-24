#include <iostream>
#include "CalculateReferenceLine.hpp"

#define SUPPRESS_LOG

int main(int argc, char* argv[]) {
	std::string filename;
	std::cout << "Enter filename (full path) to xodr file" << std::endl;
	std::cin >> filename;
	CalculateReferenceLine(filename.c_str(),0.1);


	return 0;
}
