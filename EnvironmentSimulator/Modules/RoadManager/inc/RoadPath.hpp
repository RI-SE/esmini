#pragma once

#include "Road.hpp"
#include "Position.hpp"

class RoadPath {
   public:
	typedef struct PathNode {
		RoadLink* link;
		double dist;
		Road* fromRoad;
		int fromLaneId;
		ContactPointType contactPoint;
		PathNode* previous;
	} PathNode;

	std::vector<PathNode*> visited_;
	std::vector<PathNode*> unvisited_;
	const Position* startPos_;
	const Position* targetPos_;
	int direction_;	 // direction of path from starting pos. 0==not set, 1==forward, 2==backward

	RoadPath(const Position* startPos, const Position* targetPos)
		: startPos_(startPos), targetPos_(targetPos){};
	~RoadPath();

	/**
	Calculate shortest path between starting position and target position,
	using Dijkstra's algorithm https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
	it also calculates the length of the path, or distance between the positions
	positive distance means that the shortest path was found in forward direction
	negative distance means that the shortest path goes in opposite direction from the heading of the starting
	position
	@param dist A reference parameter into which the calculated path distance is stored
	@param bothDirections Set to true in order to search also backwards from object
	@param maxDist If set the search along each path branch will terminate after reaching this distance
	@return 0 on success, -1 on failure e.g. path not found
	*/
	int Calculate(double& dist, bool bothDirections = true, double maxDist = LARGE_NUMBER);

   private:
	bool CheckRoad(Road* checkRoad, RoadPath::PathNode* srcNode, Road* fromRoad, int fromLaneId);
};
