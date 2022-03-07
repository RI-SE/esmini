#include "OverlappingReferenceLines.hpp"

OverlappingReferenceLines::OverlappingReferenceLines(const char* openDriveFileName, double deltaS)
	: CalculateReferenceLine(openDriveFileName, deltaS) {
	double error;
	std::ofstream file;
	static char strbuf[1024];
	file.open("test5.csv");

	for (const auto& [key1, value1] : openDriveReferenceLines) {
		for (const auto& [key2, value2] : openDriveReferenceLines) {
			if (key1->GetId() == key2->GetId())
				continue;
			error = rootMeanSquareError(value1, value2);

			if (error < 1 && compareGeometryHeading(key1, key2, 170.0, 190.0)) {
				std::cout << "dist error is: " << error << "for key1: " << key1->GetId()
						  << " key2: " << key2->GetId() << std::endl;
				file << "lane, " << key1->GetId() << ", " << 0 << ", " << 0 << ", driving" << std::endl;
				for (const auto v : value1) {
					snprintf(strbuf, sizeof(strbuf), "%lf, %lf, %lf, %lf\n", v->x, v->y, 0.0, 0.0);
					file << strbuf;
				}
				file << "lane, " << key2->GetId() << ", " << 0 << ", " << 0 << ", driving" << std::endl;
				for (const auto v : value2) {
					snprintf(strbuf, sizeof(strbuf), "%lf, %lf, %lf, %lf\n", v->x, v->y, 0.0, 0.0);
					file << strbuf;
				}
			}
		}
	}
}

bool OverlappingReferenceLines::compareGeometryHeading(std::shared_ptr<Road> road1,
													   std::shared_ptr<Road> road2,
													   double min,
													   double max) {
	auto geometry1 = road1->getGeometryVector();
	auto geometry2 = road2->getGeometryVector();
	double differenceHeading0 = abs(geometry1[0]->GetHdg() - geometry2[0]->GetHdg());
	double differenceHeadingEnd = abs(geometry1.back()->GetHdg() - geometry2.back()->GetHdg());
	return (differenceHeading0 > min * M_PI / 180 && differenceHeading0 < max * M_PI / 180
			&& differenceHeadingEnd > min * M_PI / 180 && differenceHeadingEnd < max * M_PI / 180);
}
double OverlappingReferenceLines::euclideanDistance(XYZ p1, XYZ p2) {
	double dist = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
	return dist;
}

double OverlappingReferenceLines::rootMeanSquareError(std::vector<std::shared_ptr<XYZ>> line1,
													  std::vector<std::shared_ptr<XYZ>> line2) {
	std::vector<double> distVector;
	for (auto p2 : line2) {
		std::vector<double> tmpDiffVector = calculateDistPoint2Line(line1, *p2);

		auto minValue = std::min_element(tmpDiffVector.begin(), tmpDiffVector.end());
		distVector.push_back(*minValue);
	}

	double distError = rootMeanSquareError(distVector);
	return distError;
}

double OverlappingReferenceLines::rootMeanSquareError(std::vector<double> errorVector) {
	double sumDist = 0;
	for (auto e : errorVector) {
		sumDist += e * e;
	}
	sumDist = sumDist / errorVector.size();
	sumDist = sqrt(sumDist);
	return sumDist;
}

std::vector<double> OverlappingReferenceLines::calculateDistPoint2Line(std::vector<std::shared_ptr<XYZ>> line,
																	   XYZ& point) {
	std::vector<double> distVector;
	double dist;
	for (auto p : line) {
		dist = euclideanDistance(*p, point);
		distVector.push_back(dist);
	}
	return distVector;
}
