
#include "ClothoidShape.hpp"

ClothoidShape::ClothoidShape(roadmanager::Position pos,
							 double curv,
							 double curvPrime,
							 double len,
							 double tStart,
							 double tEnd)
	: Shape(ShapeType::CLOTHOID) {
	pos_ = pos;
	spiral_ = new roadmanager::Spiral(0, pos_.GetX(), pos_.GetY(), pos_.GetH(), len, curv,
									  curv + curvPrime * len);
	t_start_ = tStart;
	t_end_ = tEnd;
	pline_.interpolateHeading_ = true;
}

void ClothoidShape::CalculatePolyLine() {
	// Create polyline placeholder representation
	double stepLen = 1.0;
	int steps = (int)(spiral_->GetLength() / stepLen);
	pline_.Reset();
	TrajVertex v;

	for (size_t i = 0; i < steps + 1; i++) {
		if (i < steps) {
			EvaluateInternal((double)i, v);
		} else {
			// Add endpoint of spiral
			EvaluateInternal(spiral_->GetLength(), v);
		}

		// resolve road coordinates to get elevation at point
		pos_.SetInertiaPos(v.x, v.y, v.h, true);
		v.z = pos_.GetZ();

		v.p = v.s = (double)i;
		v.time = t_start_ + (i * stepLen / spiral_->GetLength()) * t_end_;

		pline_.AddVertex(v);
	}
}

int ClothoidShape::EvaluateInternal(double s, TrajVertex& pos) {
	spiral_->EvaluateDS(s, &pos.x, &pos.y, &pos.h);

	return 0;
}

int ClothoidShape::Evaluate(double p, TrajectoryParamType ptype, TrajVertex& pos) {
	if (ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_TIME) {
		if (p >= t_start_ && p <= t_end_) {
			double t = p - t_start_;
			// Transform time parameter value into a s value
			p = GetLength() * (t - t_start_) / (t_end_ - t_start_);
		} else {
			LOG("Requested time %.2f outside range [%.2f, %.2f]", p, t_start_, t_end_);
			p = GetLength();
		}
	} else if (p > GetLength()) {
		p = GetLength();
	}

	pline_.Evaluate(p, pos);

	spiral_->EvaluateDS(p, &pos.x, &pos.y, &pos.h);

	pos.s = p;

	return 0;
}

double ClothoidShape::GetStartTime() {
	return t_start_;
}

double ClothoidShape::GetDuration() {
	return t_end_ - t_start_;
}