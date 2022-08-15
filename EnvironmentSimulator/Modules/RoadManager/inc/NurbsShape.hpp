#pragma once
#include "Shape.hpp"

class NurbsShape : public Shape {
	class ControlPoint {
	   public:
		Position pos_;
		double time_;
		double weight_;
		double t_;
		bool calcHeading_;

		ControlPoint(Position pos, double time, double weight, bool calcHeading)
			: pos_(pos), time_(time), weight_(weight), calcHeading_(calcHeading) {}
	};

   public:
	NurbsShape(int order) : order_(order), Shape(ShapeType::NURBS), length_(0) {
		pline_.interpolateHeading_ = true;
	}

	void AddControlPoint(Position pos, double time, double weight, bool calcHeading);
	void AddKnots(std::vector<double> knots);
	int Evaluate(double p, TrajectoryParamType ptype, TrajVertex& pos);
	int EvaluateInternal(double s, TrajVertex& pos);

	int order_;
	std::vector<ControlPoint> ctrlPoint_;
	std::vector<double> knot_;
	std::vector<double> d_;	 // used for temporary storage of CoxDeBoor weigthed control points
	std::vector<double>
		dPeakT_;  // used for storage of at what t value the corresponding ctrlPoint contribution peaks
	std::vector<double>
		dPeakValue_;  // used for storage of at what t value the corresponding ctrlPoint contribution peaks

	void CalculatePolyLine();
	double GetLength() { return length_; }
	double GetStartTime();
	double GetDuration();

   private:
	double CoxDeBoor(double x, int i, int p, const std::vector<double>& t);
	double length_;
};