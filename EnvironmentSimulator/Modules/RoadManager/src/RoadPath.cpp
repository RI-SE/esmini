#include "RoadPath.hpp"

bool RoadPath::CheckRoad(Road* checkRoad, RoadPath::PathNode* srcNode, Road* fromRoad, int fromLaneId) {
	// Register length of this road and find node in other end of the road (link)

	RoadLink* nextLink = 0;

	if (srcNode->link->GetElementType() == RoadLink::RoadLink::ELEMENT_TYPE_ROAD) {
		// node link is a road, find link in the other end of it
		if (srcNode->link->GetContactPointType() == ContactPointType::CONTACT_POINT_END) {
			nextLink = checkRoad->GetLink(LinkType::PREDECESSOR);
		} else {
			nextLink = checkRoad->GetLink(LinkType::SUCCESSOR);
		}
	} else if (srcNode->link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION) {
		Junction* junction = Position::GetOpenDrive()->GetJunctionById(srcNode->link->GetElementId());
		if (junction && junction->GetType() == Junction::JunctionType::DIRECT) {
			if (checkRoad->GetLink(LinkType::SUCCESSOR)
				&& checkRoad->GetLink(LinkType::SUCCESSOR)->GetElementId() == junction->GetId()) {
				// Node link is a direct junction, and it is the successor to the road being checked
				// hence next link is the predecessor of that road
				nextLink = checkRoad->GetLink(LinkType::PREDECESSOR);
			} else if (checkRoad->GetLink(LinkType::PREDECESSOR)
					   && checkRoad->GetLink(LinkType::PREDECESSOR)->GetElementId() == junction->GetId()) {
				// Node link is a direct junction, and it is the predecessor to the road being checked
				// hence next link is the successor of that road
				nextLink = checkRoad->GetLink(LinkType::SUCCESSOR);
			}
		} else {
			if (checkRoad->GetLink(LinkType::SUCCESSOR)
				&& checkRoad->GetLink(LinkType::SUCCESSOR)->GetElementId() == fromRoad->GetId()) {
				// Node link is a non direct junction, and it is the successor to the connecting road being
				// checked hence next link is the predecessor of that connecting road
				nextLink = checkRoad->GetLink(LinkType::PREDECESSOR);
			} else if (checkRoad->GetLink(LinkType::PREDECESSOR)
					   && checkRoad->GetLink(LinkType::PREDECESSOR)->GetElementId() == fromRoad->GetId()) {
				// Node link is a non direct junction, and it is the predecessor to the connecting road being
				// checked hence next link is the successor of that connecting road
				nextLink = checkRoad->GetLink(LinkType::SUCCESSOR);
			}
		}
	}

	if (nextLink == 0) {
		// end of road
		return false;
	}

	int nextLaneId = fromRoad->GetConnectingLaneId(srcNode->link, fromLaneId, checkRoad->GetId());
	if (nextLaneId == 0) {
		return false;
	}

	// Check if next node is already visited
	for (size_t i = 0; i < visited_.size(); i++) {
		if (visited_[i]->link == nextLink) {
			// Already visited, ignore and return
			return false;
		}
	}

	// Check if next node is already among unvisited
	size_t i;
	for (i = 0; i < unvisited_.size(); i++) {
		if (unvisited_[i]->link == nextLink) {
			// Consider it, i.e. calc distance and potentially store it (if less than old)
			if (srcNode->dist + checkRoad->GetLength() < unvisited_[i]->dist) {
				unvisited_[i]->dist = srcNode->dist + checkRoad->GetLength();
			}
		}
	}

	if (i == unvisited_.size()) {
		// link not visited before, add it
		PathNode* pNode = new PathNode;
		pNode->dist = srcNode->dist + checkRoad->GetLength();
		pNode->link = nextLink;
		pNode->fromRoad = checkRoad;
		pNode->fromLaneId = nextLaneId;
		pNode->previous = srcNode;
		unvisited_.push_back(pNode);
	}

	return true;
}

int RoadPath::Calculate(double& dist, bool bothDirections, double maxDist) {
	OpenDrive* odr = startPos_->GetOpenDrive();
	RoadLink* link = 0;
	Junction* junction = 0;
	Road* startRoad = odr->GetRoadById(startPos_->GetTrackId());
	Road* targetRoad = odr->GetRoadById(targetPos_->GetTrackId());
	Road* pivotRoad = startRoad;
	int pivotLaneId = startPos_->GetLaneId();
	Road* nextRoad = startRoad;
	bool found = false;
	double tmpDist = 0;
	size_t i;

	// This method will find and measure the length of the shortest path
	// between a start position and a target position
	// The implementation is based on Dijkstra's algorithm
	// https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm

	if (pivotRoad == 0) {
		LOG("Invalid startpos road ID: %d", startPos_->GetTrackId());
		return -1;
	}

	for (i = 0; i < (bothDirections ? 2 : 1); i++) {
		ContactPointType contact_point = ContactPointType::CONTACT_POINT_UNDEFINED;
		if (bothDirections) {
			if (i == 0) {
				contact_point = ContactPointType::CONTACT_POINT_START;
				link = pivotRoad->GetLink(LinkType::PREDECESSOR);  // Find link to previous road or junction
			} else {
				contact_point = ContactPointType::CONTACT_POINT_END;
				link = pivotRoad->GetLink(LinkType::SUCCESSOR);	 // Find link to previous road or junction
			}
		} else {
			// Look only in forward direction, w.r.t. entity heading
			if (startPos_->GetHRelative() < M_PI_2 || startPos_->GetHRelative() > 3 * M_PI_2) {
				// Along road direction
				contact_point = ContactPointType::CONTACT_POINT_END;
				link = pivotRoad->GetLink(LinkType::SUCCESSOR);	 // Find link to next road or junction
			} else {
				// Opposite road direction
				contact_point = ContactPointType::CONTACT_POINT_START;
				link = pivotRoad->GetLink(LinkType::PREDECESSOR);  // Find link to previous road or junction
			}
		}

		if (link) {
			PathNode* pNode = new PathNode;
			pNode->link = link;
			pNode->fromRoad = pivotRoad;
			pNode->fromLaneId = pivotLaneId;
			pNode->previous = 0;
			pNode->contactPoint = contact_point;
			if (contact_point == ContactPointType::CONTACT_POINT_START) {
				pNode->dist = startPos_->GetS();  // distance to first road link is distance to start of road
			} else if (contact_point == ContactPointType::CONTACT_POINT_END) {
				pNode->dist = pivotRoad->GetLength() - startPos_->GetS();  // distance to end of road
			}

			unvisited_.push_back(pNode);
		}
	}

	if (startRoad == targetRoad) {
		dist = targetPos_->GetS() - startPos_->GetS();

		// Special case: On same road, distance is equal to delta s
		if (startPos_->GetLaneId() < 0) {
			if (startPos_->GetHRelative() > M_PI_2 && startPos_->GetHRelative() < 3 * M_PI_2) {
				// facing opposite road direction
				dist *= -1;
			}
		} else {
			// decreasing in lanes with positive IDs
			dist *= -1;

			if (startPos_->GetHRelative() < M_PI_2 || startPos_->GetHRelative() > 3 * M_PI_2) {
				// facing along road direction
				dist *= -1;
			}
		}

		return 0;
	}

	if (unvisited_.size() == 0) {
		// No links
		dist = 0;
		return -1;
	}

	for (i = 0; i < 100 && !found && unvisited_.size() > 0 && tmpDist < maxDist; i++) {
		found = false;

		// Find unvisited PathNode with shortest distance
		double minDist = LARGE_NUMBER;
		int minIndex = 0;
		for (size_t j = 0; j < unvisited_.size(); j++) {
			if (unvisited_[j]->dist < minDist) {
				minIndex = (int)j;
				minDist = unvisited_[j]->dist;
			}
		}

		link = unvisited_[minIndex]->link;
		tmpDist = unvisited_[minIndex]->dist;
		pivotRoad = unvisited_[minIndex]->fromRoad;
		pivotLaneId = unvisited_[minIndex]->fromLaneId;

		// - Inspect all unvisited neighbor nodes (links), measure edge (road) distance to that link
		// - Note the total distance
		// - If not already in invisited list, put it there.
		// - Update distance to this link if shorter than previously registered value
		if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_ROAD) {
			// only one edge (road)
			nextRoad = odr->GetRoadById(link->GetElementId());

			if (nextRoad == targetRoad) {
				// Special case: On same road, distance is equal to delta s, direction considered
				if (link->GetContactPointType() == ContactPointType::CONTACT_POINT_START) {
					tmpDist += targetPos_->GetS();
				} else {
					tmpDist += nextRoad->GetLength() - targetPos_->GetS();
				}

				found = true;
			} else {
				CheckRoad(nextRoad, unvisited_[minIndex], pivotRoad, pivotLaneId);
			}
		} else if (link->GetElementType() == RoadLink::ElementType::ELEMENT_TYPE_JUNCTION) {
			// check all junction links (connecting roads) that has pivot road as incoming road
			junction = odr->GetJunctionById(link->GetElementId());
			for (size_t j = 0; j < junction->GetNoConnectionsFromRoadId(pivotRoad->GetId()); j++) {
				nextRoad = odr->GetRoadById(
					junction->GetConnectingRoadIdFromIncomingRoadId(pivotRoad->GetId(), (int)j));
				if (nextRoad == 0) {
					return 0;
				}

				if (nextRoad == targetRoad)	 // target road reached
				{
					ContactPointType contact_point = ContactPointType::CONTACT_POINT_UNDEFINED;
					if (nextRoad->IsSuccessor(pivotRoad, &contact_point)
						|| nextRoad->IsPredecessor(pivotRoad, &contact_point)) {
						if (contact_point == ContactPointType::CONTACT_POINT_START) {
							tmpDist += targetPos_->GetS();
						} else if (contact_point == ContactPointType::CONTACT_POINT_END) {
							tmpDist += nextRoad->GetLength() - targetPos_->GetS();
						} else {
							LOG("Unexpected contact point %s",
								OpenDrive::ContactPointType2Str(contact_point).c_str());
							return -1;
						}
					} else {
						LOG("Failed to check link in junction");
						return -1;
					}
					found = true;
				} else {
					CheckRoad(nextRoad, unvisited_[minIndex], pivotRoad, pivotLaneId);
				}
			}
		}

		// Mark pivot link as visited (move it from unvisited to visited)
		visited_.push_back(unvisited_[minIndex]);
		unvisited_.erase(unvisited_.begin() + minIndex);
	}

	if (found) {
		// Find out whether the path goes forward or backwards from starting position
		if (visited_.size() > 0) {
			RoadPath::PathNode* node = visited_.back();

			while (node) {
				if (node->previous == 0) {
					// This is the first node - inspect whether it is in front or behind start position
					if ((node->link == startRoad->GetLink(LinkType::PREDECESSOR)
						 && abs(startPos_->GetHRelative()) > M_PI_2
						 && abs(startPos_->GetHRelative()) < 3 * M_PI / 2)
						|| ((node->link == startRoad->GetLink(LinkType::SUCCESSOR)
								 && abs(startPos_->GetHRelative()) < M_PI_2
							 || abs(startPos_->GetHRelative()) > 3 * M_PI / 2))) {
						direction_ = 1;
					} else {
						direction_ = -1;
					}
				}
				node = node->previous;
			}
		}
	}

	// Compensate for heading of the start position
	if (startPos_->GetHRelativeDrivingDirection() > M_PI_2
		&& startPos_->GetHRelativeDrivingDirection() < 3 * M_PI_2) {
		direction_ *= -1;
	}
	dist = direction_ * tmpDist;

	return found ? 0 : -1;
}

RoadPath::~RoadPath() {
	for (size_t i = 0; i < visited_.size(); i++) {
		delete (visited_[i]);
	}
	visited_.clear();

	for (size_t i = 0; i < unvisited_.size(); i++) {
		delete (unvisited_[i]);
	}
	unvisited_.clear();
}
