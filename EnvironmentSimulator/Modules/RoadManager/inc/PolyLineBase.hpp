#pragma once

#include "structsandDefines.hpp"

class PolyLineBase {
   public:
	PolyLineBase()
		: length_(0), vIndex_(0), currentPos_({0, 0, 0, 0, 0, 0, 0, 0, false}), interpolateHeading_(false) {}
	TrajVertex* AddVertex(TrajVertex p);
	TrajVertex* AddVertex(double x, double y, double z, double h);
	TrajVertex* AddVertex(double x, double y, double z);

	/**
	 * Update vertex position and recalculate dependent values, e.g. length and heading
	 * NOTE: Need to be called in order, starting from i=0
	 * @param i Index of vertex to update
	 * @param x X coordinate of new position
	 * @param y Y coordinate of new position
	 * @param z Z coordinate of new position
	 * @param h Heading
	 */
	TrajVertex* UpdateVertex(int i, double x, double y, double z, double h);

	/**
	 * Update vertex position and recalculate dependent values, e.g. length and heading
	 * NOTE: Need to be called in order, starting from i=0
	 * @param i Index of vertex to update
	 * @param x X coordinate of new position
	 * @param y Y coordinate of new position
	 * @param z Z coordinate of new position
	 */
	TrajVertex* UpdateVertex(int i, double x, double y, double z);

	void reset() { length_ = 0.0; }
	int Evaluate(double s, TrajVertex& pos, double cornerRadius, int startAtIndex);
	int Evaluate(double s, TrajVertex& pos, double cornerRadius);
	int Evaluate(double s, TrajVertex& pos, int startAtIndex);
	int Evaluate(double s, TrajVertex& pos);
	int FindClosestPoint(double xin, double yin, TrajVertex& pos, int& index, int startAtIndex = 0);
	int FindPointAhead(double s_start, double distance, TrajVertex& pos, int& index, int startAtIndex = 0);
	int GetNumberOfVertices() { return (int)vertex_.size(); }
	TrajVertex* GetVertex(int index);
	void Reset();
	int Time2S(double time, double& s);

	std::vector<TrajVertex> vertex_;
	TrajVertex currentPos_;
	double length_;
	int vIndex_;
	bool interpolateHeading_;

   protected:
	int EvaluateSegmentByLocalS(int i, double local_s, double cornerRadius, TrajVertex& pos);
};
