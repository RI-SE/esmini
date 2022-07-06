#pragma once

#include <algorithm>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <map>
#include "OpenDrive.hpp"

struct RoadLineXYZ {
	std::vector<XYZ> point;
};
struct XYZ {
	double x;
	double y;
	double z;
};
struct RoadLineXYZHdg : RoadLineXYZ {
	std::vector<double> hdg;
};

class CalculateReferenceLine : public OpenDrive {
   private:
   public:
<<<<<<< Updated upstream
	CalculateReferenceLine(const char* openDriveFileName, double deltaS);
	~CalculateReferenceLine();
	std::vector<double> linspace(double length, double deltaS);
	std::vector<double> globalReflineOffset(double s, double heading);
   protected:
	std::map<int, RoadLineXYZHdg> openDriveReferenceLines;
=======
	CalculateReferenceLine(const char* openDriveFileName);
	std::vector<double> linspace(double length, double deltaS);
	std::vector<double> globalReflineOffset(double s, double heading);

   protected:
	void sampleRefline(double deltaS);
	std::map<std::shared_ptr<Road>, std::vector<std::shared_ptr<XYZ>>> openDriveReferenceLines;
>>>>>>> Stashed changes
};

/**
 * What do I need to fix, I need parampoly3 calculater and fitter same with poly3.
 * Linespace function and find nearest neighbour
 */