#pragma once

#include "Postion.hpp"
#include "Shape.hpp"
#include "structsandDefines.hpp"

class ClothoidShape : public Shape {
   public:
	ClothoidShape(roadmanager::Position pos,
				  double curv,
				  double curvDot,
				  double len,
				  double tStart,
				  double tEnd);

	int Evaluate(double p, TrajectoryParamType ptype, TrajVertex& pos);
	int EvaluateInternal(double s, TrajVertex& pos);
	void CalculatePolyLine();
	double GetLength() { return spiral_->GetLength(); }
	double GetStartTime();
	double GetDuration();

	Position pos_;
	roadmanager::Spiral* spiral_;  // make use of the OpenDRIVE clothoid definition
	double t_start_;
	double t_end_;
};