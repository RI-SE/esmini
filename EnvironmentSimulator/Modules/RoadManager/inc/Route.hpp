#pragma once

#include "Position.hpp"

// A route is a sequence of positions, at least one per road along the route
class Route {
   public:
	Route() : invalid_route_(false), waypoint_idx_(-1), path_s_(0), length_(0) {}

	/**
	Adds a waypoint to the route. One waypoint per road. At most one junction between waypoints.
	@param position A regular position created with road, lane or world coordinates
	@return Non zero return value indicates error of some kind
	*/
	int AddWaypoint(Position* position);
	int GetWayPointDirection(int index);

	void setName(std::string name);
	std::string getName();
	double GetLength() { return length_; }
	void CheckValid();
	bool IsValid() { return !invalid_route_; }

	// Current route position data
	// Actual object position might differ, e.g. laneId or even trackId in junctions
	double GetTrackS() { return currentPos_.GetS(); }
	double GetPathS() { return path_s_; }
	int GetLaneId() { return currentPos_.GetLaneId(); }
	int GetTrackId() { return currentPos_.GetTrackId(); }
	Position* GetWaypoint(int index = -1);	// -1 means current
	Road* GetRoadAtOtherEndOfConnectingRoad(Road* incoming_road);
	int GetDirectionRelativeRoad();
	Position* GetCurrentPosition() { return &currentPos_; }

	/**
	Specify route position in terms of a track ID and track S value
	@return Non zero return value indicates error of some kind
	*/
	Position::ErrorCode SetTrackS(int trackId, double s);

	/**
	Move current position forward, or backwards, ds meters along the route
	@param ds Distance to move, negative will move backwards
	@param actualDistance Distance considering lateral offset and curvature (true/default) or along centerline
	(false)
	@return Non zero return value indicates error of some kind, most likely End Of Route
	*/
	Position::ErrorCode MovePathDS(double ds);

	/**
	Move current position to specified S-value along the route
	@param route_s Distance to move, negative will move backwards
	@return Non zero return value indicates error of some kind, most likely End Of Route
	*/
	Position::ErrorCode SetPathS(double s);

	Position::ErrorCode CopySFractionOfLength(Position* pos);

	std::vector<Position> minimal_waypoints_;  // used only for the default controllers
	std::vector<Position> all_waypoints_;	   // used for user-defined controllers
	std::string name_;
	bool invalid_route_;
	double path_s_;
	Position currentPos_;
	double length_;
	int waypoint_idx_;
};
