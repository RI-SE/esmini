#pragma once

#include <iostream>
#include <map>
#include "OpenDrive.hpp"
class CalculateReferenceLine : public OpenDrive {
   private:
   public:
	CalculateReferenceLine(const char* openDriveFileName);
	// ~CalculateReferenceLine();
};

struct RoadLine3D {
	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> z;
	
};


/**
 * What do I need to fix, I need parampoly3 calculater and fitter same with poly3.
 * Linespace function and find nearest neighbour
 */