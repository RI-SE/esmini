#include "Route.hpp"

int Route::AddWaypoint(Position* position) {
	if (minimal_waypoints_.size() > 0) {
		// Keep only one consecutive waypoint per road
		// Keep first specified waypoint for first road
		// then, for following roads, keep the last waypoint.

		if (position->GetTrackId() == minimal_waypoints_.back().GetTrackId()) {
			if (minimal_waypoints_.size() == 1) {
				// Ignore
				LOG("Ignoring additional waypoint for road %d (s %.2f)", position->GetTrackId(),
					position->GetS());
				all_waypoints_.push_back(*position);
				return -1;
			} else	// at least two road-unique waypoints
			{
				// Keep this, remove previous
				LOG("Removing previous waypoint for same road %d (at s %.2f)",
					minimal_waypoints_.back().GetTrackId(), minimal_waypoints_.back().GetS());
				minimal_waypoints_.pop_back();
			}
		}

		// Check that there is a valid path from previous waypoint
		RoadPath* path = new RoadPath(&minimal_waypoints_.back(), position);
		double dist = 0;

		if (path->Calculate(dist, false) == 0) {
			// Path is found by tracing previous nodes
			RoadPath::PathNode* previous = 0;
			std::vector<RoadPath::PathNode*> nodes;

			if (path->visited_.size() > 0) {
				previous = path->visited_.back()->previous;
				nodes.push_back(path->visited_.back());
				while (previous != nullptr) {
					nodes.push_back(previous);
					previous = previous->previous;
				}
			}

			if (nodes.size() > 1) {
				// Add internal waypoints, one for each road along the path
				for (int i = (int)nodes.size() - 1; i >= 1; i--) {
					// Find out lane ID of the connecting road
					Position connected_pos
						= Position(nodes[i - 1]->fromRoad->GetId(), nodes[i - 1]->fromLaneId, 0, 0);
					all_waypoints_.push_back(*position);
					minimal_waypoints_.push_back(connected_pos);
					LOG("Route::AddWaypoint Added intermediate waypoint %d roadId %d laneId %d",
						(int)minimal_waypoints_.size() - 1, connected_pos.GetTrackId(),
						nodes[i - 1]->fromLaneId);
				}
			}

			length_ += dist;
		} else {
			invalid_route_ = true;
		}
	} else {
		// First waypoint, make it the current position
		currentPos_ = *position;
	}
	all_waypoints_.push_back(*position);
	minimal_waypoints_.push_back(*position);
	LOG("Route::AddWaypoint Added waypoint %d: %d, %d, %.2f", (int)minimal_waypoints_.size() - 1,
		position->GetTrackId(), position->GetLaneId(), position->GetS());

	return 0;
}

void Route::CheckValid() {
	if (invalid_route_) {
		LOG("Warning: Route %s is not valid, will be ignored for the default controller.", getName().c_str());
		minimal_waypoints_.clear();
	}
}

Road* Route::GetRoadAtOtherEndOfConnectingRoad(Road* incoming_road) {
	Road* connecting_road = Position::GetOpenDrive()->GetRoadById(GetTrackId());
	Junction* junction = Position::GetOpenDrive()->GetJunctionById(connecting_road->GetJunction());

	if (junction == 0) {
		LOG("Unexpected: Road %d not a connecting road", connecting_road->GetId());
		return 0;
	}

	return junction->GetRoadAtOtherEndOfConnectingRoad(connecting_road, incoming_road);
}

int Route::GetDirectionRelativeRoad() {
	return GetWayPointDirection(waypoint_idx_);
}

int Route::GetWayPointDirection(int index) {
	if (minimal_waypoints_.size() == 0 || index < 0 || index >= minimal_waypoints_.size()) {
		LOG("Waypoint index %d out of range (%d)", index, minimal_waypoints_.size());
		return 0;
	}

	if (minimal_waypoints_.size() == 1) {
		LOG("Only one waypoint, no direction");
		return 0;
	}

	OpenDrive* od = minimal_waypoints_[index].GetOpenDrive();
	Road* road = od->GetRoadById(minimal_waypoints_[index].GetTrackId());
	if (road == nullptr) {
		LOG("Waypoint %d invalid road id %d!", index, minimal_waypoints_[index].GetTrackId());
		return 0;
	}

	int direction = 0;
	Position* pos2 = nullptr;

	// Looking in the direction of heading
	direction
		= minimal_waypoints_[index].GetHRelative() > M_PI_2 && currentPos_.GetHRelative() < 3 * M_PI_2 / 2.0
			  ? -1
			  : 1;

	if (index < minimal_waypoints_.size() - 1) {
		// Looking in the direction of heading
		direction
			= currentPos_.GetHRelative() > M_PI_2 && currentPos_.GetHRelative() < 3 * M_PI_2 / 2.0 ? -1 : 1;

		// Look at next waypoint
		pos2 = GetWaypoint(index + 1);
	} else if (index > 0) {
		// Looking in the opposite direction of heading
		direction
			= currentPos_.GetHRelative() > M_PI_2 && currentPos_.GetHRelative() < 3 * M_PI_2 / 2.0 ? 1 : -1;

		// Look at previous waypoint
		pos2 = GetWaypoint(index - 1);
	}

	if (direction == 1 && road->IsSuccessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId()))
		|| direction == -1
			   && road->IsPredecessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId()))) {
		// Expected case, route direction aligned with waypoint headings
		return 1;
	} else if (road->IsSuccessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId()))
			   && road->IsPredecessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId()))) {
		LOG("Road %d connects to both ends of road %d using relative heading of waypoint", pos2->GetTrackId(),
			road->GetId());
		return direction;
	} else if (road->IsSuccessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId()))) {
		return 1 * direction;
	} else if (road->IsPredecessor(Position::GetOpenDrive()->GetRoadById(pos2->GetTrackId()))) {
		return -1 * direction;
	}

	LOG("Unexpected case, failed to find out direction of route (from road id %d)", road->GetId());

	return direction;
}

void Route::setName(std::string name) {
	this->name_ = name;
}

std::string Route::getName() {
	return name_;
}
