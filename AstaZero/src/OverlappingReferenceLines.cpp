#include "OverlappingReferenceLines.hpp"

OverlappingReferenceLines::OverlappingReferenceLines(const char* openDriveFileName, double deltaS)
	: CalculateReferenceLine(openDriveFileName, deltaS) {
	double error;
	std::string output_file_name = "test2.csv";
	std::ofstream file;
	file.open(output_file_name);
	std::string sampling_step = "1.0";
	static char strbuf[1024];
	for (auto it = openDriveReferenceLines.begin(); it != openDriveReferenceLines.end(); ++it) {
		if (std::next(it, 1) == openDriveReferenceLines.end())
			break;
		error = rootMeanSquareError(it->second, (std::next(it, 1)->second));
		if (error < 1 && compareGeometryHeading(it->first, std::next(it, 1)->first, 170.0, 190.0)) {
			file << "lane, " << it->first->GetId() << ", " << 0 << ", " << 0 << ", driving" << std::endl;

			std::cout << "Error Dist: " << error << " Road id1 :" << it->first->GetId()
					  << " Road id2: " << std::next(it, 1)->first->GetId() << std::endl;
			for (auto v : it->second) {
				snprintf(strbuf, sizeof(strbuf), "%f, %f, %f, %f\n", v->x, v->y, 0, 0);
				file << strbuf;
			}
			file << "lane, " << std::next(it, 1)->first->GetId() << ", " << 0 << ", " << 0 << ", driving"
				 << std::endl;

			for (auto v : std::next(it, 1)->second) {
				snprintf(strbuf, sizeof(strbuf), "%f, %f, %f, %f\n", v->x, v->y, 0, 0);
				file << strbuf;
			}
			overlappingRoads.insert(
				std::pair<std::shared_ptr<Road>, std::shared_ptr<Road>>(it->first, std::next(it, 1)->first));
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
