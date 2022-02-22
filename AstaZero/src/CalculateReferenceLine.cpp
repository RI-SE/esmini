#include "CalculateReferenceLine.hpp"

CalculateReferenceLine::CalculateReferenceLine(const char* openDriveFileName) 
	: OpenDrive(openDriveFileName) {
	
	for (auto r : road_) {
		for (auto ls : r->getLaneSectionVector()) {
			for (auto l : ls->getLaneVector()) {
				for (auto rm : l->getLaneRoadMarkVector()) {
					std::cout << "road mark for lane "
							  // << lane->GetId() << " has width "
							  << rm->GetWidth() << std::endl;
				}
			}
		}
	}
}

// CalculateReferenceLine::~CalculateReferenceLine() {}
