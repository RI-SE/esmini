#include "OverlappingReferenceLines.hpp"
OverlappingReferenceLines::OverlappingReferenceLines(const char* openDriveFileName, double deltaS)
	: CalculateReferenceLine(openDriveFileName, deltaS) {
	double error;
	std::ofstream file;
	static char strbuf[1024];
	file.open("test3.csv");

	for (const auto& [key1, value1] : openDriveReferenceLines) {
		for (const auto& [key2, value2] : openDriveReferenceLines) {
			if (key1 == key2)
				continue;
			error = rootMeanSquareError(value1, value2);
			if (error < 0.5) {
				std::cout << "error is: " << error << "for key1: " << key1 << " key2: " << key2 << std::endl;
				file << "lane, " << key1 << ", " << 0 << ", " << 0 << ", driving" << std::endl;
				for (const auto v : value1.point) {
					snprintf(strbuf, sizeof(strbuf), "%lf, %lf, %lf, %lf\n", v.x, v.y, 0.0, 0.0);
					file << strbuf;
				}
				file << "lane, " << key2 << ", " << 0 << ", " << 0 << ", driving" << std::endl;
				for (const auto v: value2.point) {
					snprintf(strbuf, sizeof(strbuf), "%lf, %lf, %lf, %lf\n", v.x, v.y, 0.0, 0.0);
					file << strbuf;
				}
			}
		}
	}
}

OverlappingReferenceLines::~OverlappingReferenceLines() {}

double OverlappingReferenceLines::euclideanDistance(XYZ p1, XYZ p2) {
	double dist = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
	return dist;
}

double OverlappingReferenceLines::rootMeanSquareError(RoadLineXYZ line1, RoadLineXYZ line2) {
	std::vector<double> distVector;
	for (auto p2 : line2.point) {
		std::vector<double> tmpDistVector = distPoint2Line(line1, p2);
		auto minValue = std::min_element(tmpDistVector.begin(), tmpDistVector.end());
		distVector.push_back(*minValue);
	}
	double error = rootMeanSquareError(distVector);
	return error;
}

double OverlappingReferenceLines::rootMeanSquareError(std::vector<double> errorVector) {
	double sum = 0;
	for (auto e : errorVector) {
		sum += e * e;
	}
	sum = sum / errorVector.size();
	sum = sqrt(sum);
}
// lambda expression that calculate complete vector using euclidian dstnae

std::vector<double> OverlappingReferenceLines::distPoint2Line(RoadLineXYZ line, XYZ point) {
	std::vector<double> distVector;
	double dist;
	for (auto p : line.point) {
		dist = euclideanDistance(p, point);
		distVector.push_back(dist);
	}
	return distVector;
}
