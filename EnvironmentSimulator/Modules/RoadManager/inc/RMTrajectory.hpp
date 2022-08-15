#pragma once
#include "Shape.hpp"

class RMTrajectory {
   public:
	Shape* shape_;

	RMTrajectory(Shape* shape, std::string name, bool closed) : shape_(shape), name_(name), closed_(closed) {}
	RMTrajectory() : shape_(0), closed_(false) {}
	void Freeze();
	double GetLength() { return shape_ ? shape_->GetLength() : 0.0; }
	double GetTimeAtS(double s);
	double GetStartTime();
	double GetDuration();

	std::string name_;
	bool closed_;
};
