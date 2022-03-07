#pragma once

#include <algorithm>
#include <cassert>
#include <iomanip>
#include <iostream>
#include <map>
#include "OpenDrive.hpp"

class XYZ {
   public:
	double x;
	double y;
	double z;
};
class XYZHdg : public XYZ {
   public:
	double heading;
};

class CalculateReferenceLine : public OpenDrive {
   private:
   public:
	CalculateReferenceLine(const char* openDriveFileName, double deltaS);
	std::vector<double> linspace(double length, double deltaS);
	std::vector<double> globalReflineOffset(double s, double heading);

   protected:
	std::map<std::shared_ptr<Road>, std::vector<std::shared_ptr<XYZ>>> openDriveReferenceLines;
};
