#include "CalculateReferenceLine.hpp"

CalculateReferenceLine::CalculateReferenceLine(const char* openDriveFileName, double deltaS)
	: OpenDrive(openDriveFileName) {
	static char strbuf[1024];
	double x, y, hdg, laneoff;
	XYZ tmpXYZ;


	for (auto r : road_) {
		// file << "lane, " << r->GetId() << ", " << 0 << ", " << 0 << ", driving" << std::endl;
		RoadLineXYZHdg roadRefLine;
		for (auto g : r->getGeometryVector()) {
			auto lin = linspace(g->GetLength(), deltaS);
			for (auto ds : lin) {
				g->EvaluateDS(ds, &x, &y, &hdg);
				laneoff = r->GetLaneOffset(ds + g->GetS());
				auto reflineOffsetXY = globalReflineOffset(laneoff, hdg);

				x += reflineOffsetXY[0];
				y += reflineOffsetXY[1];

				// snprintf(strbuf, sizeof(strbuf), "%lf, %lf, %lf, %lf\n", x, y, 0.0, hdg);
				// file << strbuf;
				tmpXYZ.x = x;
				tmpXYZ.y = y;
				tmpXYZ.z = 0.0;
				roadRefLine.point.push_back(tmpXYZ);
				roadRefLine.hdg.push_back(hdg);
			}
		}
		openDriveReferenceLines.insert(std::pair<int, RoadLineXYZHdg>(r->GetId(), roadRefLine));
	}
}
CalculateReferenceLine::~CalculateReferenceLine() {}

std::vector<double> CalculateReferenceLine::linspace(double length, double deltaS) {
	size_t nrOfPoints = size_t(length / deltaS);
	std::vector<double> linspace(nrOfPoints + 1);
	std::generate(linspace.begin(), linspace.end(), [=, n = 0]() mutable {
		double q = n * deltaS;
		n++;
		return q;
	});
	if (linspace.back() < length) {
		linspace.push_back(length);

	} else if (linspace[nrOfPoints] < length) {
		assert(false && "linspace end value heigher than road lenght, should be impossible to reach");
		// should not be abel to get here
		auto it = std::remove_if(linspace.begin(), linspace.end(), [=](double i) { return (i > length); });
		linspace.erase(it);
		linspace.push_back(length);
	}

	return linspace;
}
// CalculateReferenceLine::~CalculateReferenceLine() {}
std::vector<double> CalculateReferenceLine::globalReflineOffset(double offsetT, double hdg) {
	double offsetX = offsetT * cos(hdg + M_PI_2);
	double offsetY = offsetT * sin(hdg + M_PI_2);
	std::vector<double> v = {offsetX, offsetY};
	return v;
}