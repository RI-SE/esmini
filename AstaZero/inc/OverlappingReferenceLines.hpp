
#pragma once

#include <cmath>
#include "CalculateReferenceLine.hpp"

class OverlappingReferenceLines : public CalculateReferenceLine {
   public:
	OverlappingReferenceLines(const char* openDriveFileName, double deltaS);
	double euclideanDistance(XYZ p1, XYZ p2);
	double rootMeanSquareError(std::vector<std::shared_ptr<XYZ>> line1,
							   std::vector<std::shared_ptr<XYZ>> line2);
	double rootMeanSquareError(std::vector<double> errorVector);
	std::vector<double> calculateDistPoint2Line(std::vector<std::shared_ptr<XYZ>> line, XYZ& point);
	bool compareGeometryHeading(std::shared_ptr<Road> r1, std::shared_ptr<Road> r2, double min, double max);
	void printOverlappingRoads();
	std::string outputFile = "overlappingReflines.csv";

   protected:
	std::map<std::shared_ptr<Road>, std::shared_ptr<Road>> overlappingRoads;
	void calculateRoadsOvelapping();

   private:
	void writeOverlappingReflinetofile(std::map<std::shared_ptr<Road>, std::vector<std::shared_ptr<XYZ>>>::iterator it);
	std::ofstream file;
};
