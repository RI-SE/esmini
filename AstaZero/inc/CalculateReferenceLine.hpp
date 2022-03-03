#pragma once

#include <algorithm>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <map>
#include "OpenDrive.hpp"

struct RoadLineXYZ {
	std::vector<double> x;
	std::vector<double> y;
	std::vector<double> z;
};
struct RoadLineXYZHdg {
	RoadLineXYZ xyz;
	std::vector<double> hdg;
};

class CalculateReferenceLine : public OpenDrive {
   private:
   public:
	CalculateReferenceLine(const char* openDriveFileName, double deltaS);
	~CalculateReferenceLine();
	std::vector<double> linspace(double length, double deltaS);
	std::vector<double> globalReflineOffset(double s, double heading);
	// ~CalculateReferenceLine();
   protected:
	std::map<int, RoadLineXYZHdg> openDriveReferenceLines;
};



/**
 * What do I need to fix, I need parampoly3 calculater and fitter same with poly3.
 * Linespace function and find nearest neighbour
 */