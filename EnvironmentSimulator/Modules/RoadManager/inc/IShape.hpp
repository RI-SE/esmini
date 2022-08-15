#pragma once

#include "structsandDefines.hpp"

class Shape {
   public:
	typedef enum { POLYLINE, CLOTHOID, NURBS, SHAPE_TYPE_UNDEFINED } ShapeType;

	typedef enum { TRAJ_PARAM_TYPE_S, TRAJ_PARAM_TYPE_TIME } TrajectoryParamType;

	Shape(ShapeType type) : type_(type) {}
	virtual int Evaluate(double p, TrajectoryParamType ptype, TrajVertex& pos) { return -1; };
	int FindClosestPoint(double xin, double yin, TrajVertex& pos, int& index, int startAtIndex = 0) {
		return -1;
	};
	virtual double GetLength() { return 0.0; }
	virtual double GetStartTime() { return 0.0; }
	virtual double GetDuration() { return 0.0; }
	ShapeType type_;

	PolyLineBase pline_;  // approximation of shape, used for calculations and visualization
};