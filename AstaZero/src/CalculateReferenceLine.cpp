#include "CalculateReferenceLine.hpp"

CalculateReferenceLine::CalculateReferenceLine(const char* openDriveFileName)
	: OpenDrive(openDriveFileName) {}

void CalculateReferenceLine::sampleRefline(double deltaS) {
	double x, y, hdg, laneoff;
	XYZ tmpXYZ;


	for (auto r : road_) {
		std::vector<std::shared_ptr<XYZ>> roadRefLine;

		for (auto g : r->getGeometryVector()) {
			auto lin = linspace(g->GetLength(), deltaS);
			for (auto ds : lin) {
				g->EvaluateDS(ds, &x, &y, &hdg);
				laneoff = r->GetLaneOffset(ds + g->GetS());
				auto reflineOffsetXY = globalReflineOffset(laneoff, hdg);

				x += reflineOffsetXY[0];
				y += reflineOffsetXY[1];

				tmpXYZHdg.x = x;
				tmpXYZHdg.y = y;
				tmpXYZHdg.z = 0.0;
				tmpXYZHdg.heading = hdg;
				roadRefLine.push_back(std::make_shared<XYZHdg>(tmpXYZHdg));
			}
		}
		openDriveReferenceLines.insert(std::pair<int, RoadLineXYZHdg>(r->GetId(), roadRefLine));
	}
}
CalculateReferenceLine::~CalculateReferenceLine() {}

std::vector<double> CalculateReferenceLine::linspace(double length, double deltaS) {
	size_t nrOfSegments = size_t(length / deltaS);
	std::vector<double> linspace(nrOfSegments + 1);
	std::generate(linspace.begin(), linspace.end(), [=, n = 0]() mutable {
		double q = n * deltaS;
		n++;
		return q;
	});
	if (linspace.back() < length) {
		linspace.push_back(length);

	} else if (linspace[nrOfSegments] < length) {
		assert(false && "linspace end value heigher than road lenght, should be impossible to reach");
	}

	return linspace;
}
std::vector<double> CalculateReferenceLine::globalReflineOffset(double offsetT, double hdg) {
	double offsetX = offsetT * cos(hdg + M_PI_2);
	double offsetY = offsetT * sin(hdg + M_PI_2);
	std::vector<double> v = {offsetX, offsetY};
	return v;
}