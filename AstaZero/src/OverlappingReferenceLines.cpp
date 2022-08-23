#include "OverlappingReferenceLines.hpp"

OverlappingReferenceLines::OverlappingReferenceLines(const char* openDriveFileName, double deltaS)
	: CalculateReferenceLine(openDriveFileName, deltaS) {
	calculateRoadsOvelapping();
}

/**
 * @brief Check if the roads referenceline are overlapping
 * if so add the roads object to overlappingroads map as pairs
 */
void OverlappingReferenceLines::calculateRoadsOvelapping() {
	double error, minError = 1.0, minHeading = 170.0, maxHeading = 190.0;
	for (auto it = openDriveReferenceLines.begin(); it != openDriveReferenceLines.end(); ++it) {
		if (std::next(it, 1) == openDriveReferenceLines.end())
			break;
		error = rootMeanSquareError(it->second, (std::next(it, 1)->second));
		if (error < minError
			&& compareGeometryHeading(it->first, std::next(it, 1)->first, minHeading, maxHeading)) {
			writeOverlappingReflinetofile(it);
			overlappingRoads.insert(
				std::pair<std::shared_ptr<Road>, std::shared_ptr<Road>>(it->first, std::next(it, 1)->first));
		}
	}
	printOverlappingRoads();
}
/**
 * @brief  compare heading of two roads and if within min and max bounderies return true
 * @param road1 vector with Road class pointers
 * @param road2 vector with Road class pointers
 * @param minDeg double value in degrees for acceptable minimum missalignment in heading when comapring road
 * reference lines
 * @param maxDeg double value in degrees for acceptable maximum missalignment in heading when comapring road
 * reference lines
 * @return true	if roads are overlapping
 * @return false if roads are not overlapping
 */
bool OverlappingReferenceLines::compareGeometryHeading(std::shared_ptr<Road> road1,
													   std::shared_ptr<Road> road2,
													   double minDeg,
													   double maxDeg) {
	auto geometry1 = road1->getGeometryVector();
	auto geometry2 = road2->getGeometryVector();
	double differenceHeading0 = abs(geometry1[0]->GetHdg() - geometry2[0]->GetHdg());
	double minRad = minDeg * M_PI / 180;
	double maxRad = maxDeg * M_PI / 180;
	double differenceHeadingEnd = abs(geometry1.back()->GetHdg() - geometry2.back()->GetHdg());
	return (differenceHeading0 > minRad && differenceHeading0 < maxRad && differenceHeadingEnd > minRad
			&& differenceHeadingEnd < maxRad);
}

/**
 * @brief calcualte euclidian distance between two points
 * @param p1 XYZ point
 * @param p2 XYZ point
 * @return double Euclidean distance between two points
 */
double OverlappingReferenceLines::euclideanDistance(XYZ p1, XYZ p2) {
	double dist = sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2) + pow(p1.z - p2.z, 2));
	return dist;
}

/**
 * @brief calculate root mean square error between two lines
 * @param line1 line of XYZ points
 * @param line2 line of XYZ points
 * @return double, root mean square error between line1 and line2
 */
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
/**
 * @brief calculate root mean square error of a vector of values
 * @param errorVector vector of values
 * @return double, root mean square error of a vector of values
 */
double OverlappingReferenceLines::rootMeanSquareError(std::vector<double> errorVector) {
	double sumDist = 0;
	for (auto e : errorVector) {
		sumDist += e * e;
	}
	sumDist = sumDist / errorVector.size();
	sumDist = sqrt(sumDist);
	return sumDist;
}
/**
 * @brief calculate distance between a point and a line
 * @param line line of XYZ points
 * @param point point of XYZ
 * @return std::vector<double>
 */
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
/**
 * @brief write overlapping referenceline to terminal
 *
 */
void OverlappingReferenceLines::printOverlappingRoads() {
	std::cout << "Overlapping Roads: " << std::endl;
	for (auto r : overlappingRoads) {
		std::cout << "Road id1: " << r.first->GetId() << " Road id2: " << r.second->GetId() << std::endl;
	}
}
/**
 * @brief Write the roads that are overlapping with corresponding refline to file that can be inputed to
 * odrplot xodr.py file for grafic visualization
 */
void OverlappingReferenceLines::writeOverlappingReflinetofile(
	std::map<std::shared_ptr<Road>, std::vector<std::shared_ptr<XYZ>>>::iterator it) {
	std::string sampling_step = "1.0";

	file.open(outputFile);
	file << "lane, " << it->first->GetId() << ", " << 0 << ", " << 0 << ", driving" << std::endl;

	for (auto v : it->second) {
		file << v->x << ", " << v->y << ", " << 0 << ", " << 0 << ", " << std::endl;
	}
	file << "lane, " << std::next(it, 1)->first->GetId() << ", " << 0 << ", " << 0 << ", driving"
		 << std::endl;

	for (auto v : std::next(it, 1)->second) {
		file << v->x << ", " << v->y << ", " << 0 << ", " << 0 << ", " << std::endl;
	}

	file.close();
}