#include "CalculateReferenceLine.hpp"

CalculateReferenceLine::CalculateReferenceLine(const char* openDriveFileName, double deltaS)
	: OpenDrive(openDriveFileName) {
	// std::ofstream file;
	// file.open("test.csv");
	static char strbuf[1024];
	double x, y, hdg, laneoff;
	XYZHdg tmpXYZHdg;


	for (auto r : road_) {
		// file << "lane, " << r->GetId() << ", " << 0 << ", " << 0 << ", driving" << std::endl;
		std::vector<std::shared_ptr<XYZ>> roadRefLine;
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
				tmpXYZHdg.x = x;
				tmpXYZHdg.y = y;
				tmpXYZHdg.z = 0.0;
				tmpXYZHdg.heading = hdg;
				roadRefLine.push_back(std::make_shared<XYZHdg>(tmpXYZHdg));
			}
		}
		openDriveReferenceLines.insert(std::pair<std::shared_ptr<Road>, std::vector<std::shared_ptr<XYZ>>>(r, roadRefLine));
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