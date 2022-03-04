#include <cmath>
#include "CalculateReferenceLine.hpp"
class OverlappingReferenceLines : public CalculateReferenceLine {
   private:
	/* data */
   public:
	OverlappingReferenceLines(const char* openDriveFileName, double deltaS);
	~OverlappingReferenceLines();
	double euclideanDistance(XYZ p1, XYZ p2);
	double rootMeanSquareError(RoadLineXYZ line1, RoadLineXYZ line2);
	double rootMeanSquareError(std::vector<double> errorVector);
	std::vector<double> distPoint2Line(RoadLineXYZ line, XYZ point);
};
