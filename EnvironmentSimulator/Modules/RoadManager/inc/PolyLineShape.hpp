#pragma once

#include "Shape.hpp"

class PolyLineShape : public Shape {
   public:
	class Vertex {
	   public:
		Position pos_;
	};

	PolyLineShape() : Shape(ShapeType::POLYLINE) {}
	void AddVertex(Position pos, double time, bool calculateHeading);
	int Evaluate(double p, TrajectoryParamType ptype, TrajVertex& pos);
	double GetLength() { return pline_.length_; }
	double GetStartTime();
	double GetDuration();

	std::vector<Vertex*> vertex_;
};
