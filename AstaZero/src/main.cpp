#include <iostream>
#include "OverlappingReferenceLines.hpp"

#define SUPPRESS_LOG

int main(int argc, char* argv[]) {
	std::string filename;
	std::cout << "Enter filename (full path) to xodr file" << std::endl;
	std::cin >> filename;
	OverlappingReferenceLines(filename.c_str(),1);


	return 0;
}
