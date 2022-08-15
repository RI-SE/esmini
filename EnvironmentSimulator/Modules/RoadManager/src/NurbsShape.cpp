#include "NurbsShape.hpp"

double NurbsShape::CoxDeBoor(double x, int i, int k, const std::vector<double>& t) {
	// Inspiration: Nurbs Curve Example @
	// https://nccastaff.bournemouth.ac.uk/jmacey/OldWeb/RobTheBloke/www/opengl_programming.html

	if (k == 1) {
		if (t[i] <= x && x < t[i + 1]) {
			return 1.0;
		}
		return 0.0;
	}

	double den1 = t[i + k - 1] - t[i];
	double den2 = t[i + k] - t[i + 1];
	double eq1 = 0.0;
	double eq2 = 0.0;

	if (den1 > 0) {
		eq1 = ((x - t[i]) / den1) * CoxDeBoor(x, i, k - 1, t);
	}

	if (den2 > 0) {
		eq2 = (t[i + k] - x) / den2 * CoxDeBoor(x, i + 1, k - 1, t);
	}

	return eq1 + eq2;
}

void NurbsShape::CalculatePolyLine() {
	if (ctrlPoint_.size() < 1) {
		return;
	}
	Position tmpRoadPos;

	// Calculate approximate length - to find a reasonable step length

	length_ = 0;
	double steplen = 1.0;  // steplen in meters
	for (size_t i = 0; i < ctrlPoint_.size(); i++) {
		ctrlPoint_[i].pos_.ReleaseRelation();
		ctrlPoint_[i].t_ = knot_[i + order_ - 1];
		if (i > 0) {
			length_ += PointDistance2D(ctrlPoint_[i - 1].pos_.GetX(), ctrlPoint_[i - 1].pos_.GetY(),
									   ctrlPoint_[i].pos_.GetX(), ctrlPoint_[i].pos_.GetY());
		}
	}

	if (length_ == 0) {
		throw std::runtime_error("Nurbs zero length - check controlpoints");
	}

	// Calculate arc length
	double newLength = 0.0;
	int nSteps = (int)(1 + length_ / steplen);
	double p_steplen = knot_.back() / nSteps;
	TrajVertex pos = {0, 0, 0, 0, 0, 0, 0, 0, false};
	TrajVertex oldpos = {0, 0, 0, 0, 0, 0, 0, 0, false};
	TrajVertex tmppos = {0, 0, 0, 0, 0, 0, 0, 0, false};

	pline_.Reset();
	for (int i = 0; i < nSteps + 1; i++) {
		double t = i * p_steplen;
		EvaluateInternal(t, pos);

		// Calulate heading from line segment between this and previous vertices
		if (i < nSteps) {
			EvaluateInternal(t + 0.01 * p_steplen, tmppos);
		} else {
			EvaluateInternal(t - 0.01 * p_steplen, tmppos);
		}

		if (PointDistance2D(tmppos.x, tmppos.y, pos.x, pos.y) < SMALL_NUMBER) {
			// If points conside, use heading from polyline
			pos.calcHeading = false;
		} else {
			if (i < nSteps) {
				pos.h = GetAngleInInterval2PI(atan2(tmppos.y - pos.y, tmppos.x - pos.x));
			} else {
				pos.h = GetAngleInInterval2PI(atan2(pos.y - tmppos.y, pos.x - tmppos.x));
			}
		}

		if (i > 0) {
			newLength += PointDistance2D(pos.x, pos.y, oldpos.x, oldpos.y);
		}
		pos.s = newLength;

		// Find max contributing controlpoint for time interpolation
		for (int j = 0; j < ctrlPoint_.size(); j++) {
			if (d_[j] > dPeakValue_[j]) {
				dPeakValue_[j] = d_[j];
				dPeakT_[j] = t;
			}
		}

		pline_.AddVertex(pos);
		pline_.vertex_[i].p = i * p_steplen;
		oldpos = pos;
		// Resolve Z value - from road elevation
		tmpRoadPos.SetInertiaPos(pos.x, pos.y, pos.h);
		pos.z = tmpRoadPos.GetZ();
		pline_.vertex_[i].z = pos.z;
	}

	// Calculate time interpolations
	int currentCtrlPoint = 0;
	for (int i = 0; i < pline_.vertex_.size(); i++) {
		if (pline_.vertex_[i].p >= dPeakT_[currentCtrlPoint + 1]) {
			currentCtrlPoint = MIN(currentCtrlPoint + 1, (int)(ctrlPoint_.size()) - 2);
		}
		double w = (pline_.vertex_[i].p - dPeakT_[currentCtrlPoint])
				   / (dPeakT_[currentCtrlPoint + 1] - dPeakT_[currentCtrlPoint]);
		pline_.vertex_[i].time
			= ctrlPoint_[currentCtrlPoint].time_
			  + w * (ctrlPoint_[currentCtrlPoint + 1].time_ - ctrlPoint_[currentCtrlPoint].time_);
	}

	length_ = newLength;
}

int NurbsShape::EvaluateInternal(double t, TrajVertex& pos) {
	pos.x = pos.y = 0.0;

	// Find knot span
	t = CLAMP(t, knot_[0], knot_.back() - SMALL_NUMBER);

	double rationalWeight = 0.0;

	for (size_t i = 0; i < ctrlPoint_.size(); i++) {
		// calculate the effect of this point on the curve
		d_[i] = CoxDeBoor(t, (int)i, order_, knot_);
		rationalWeight += d_[i] * ctrlPoint_[i].weight_;
	}

	for (size_t i = 0; i < ctrlPoint_.size(); i++) {
		if (d_[i] > SMALL_NUMBER) {
			// sum effect of CV on this part of the curve
			pos.x += d_[i] * ctrlPoint_[i].pos_.GetX() * ctrlPoint_[i].weight_ / rationalWeight;
			pos.y += d_[i] * ctrlPoint_[i].pos_.GetY() * ctrlPoint_[i].weight_ / rationalWeight;
		}
	}

	return 0;
}

void NurbsShape::AddControlPoint(Position pos, double time, double weight, bool calcHeading) {
	if (calcHeading == false) {
		LOG_ONCE("Info: Explicit orientation in Nurbs trajectory control points not supported yet");
	}
	ctrlPoint_.push_back(ControlPoint(pos, time, weight, true));
	d_.push_back(0);
	dPeakT_.push_back(0);
	dPeakValue_.push_back(0);
}

void NurbsShape::AddKnots(std::vector<double> knots) {
	knot_ = knots;

	if (knot_.back() < SMALL_NUMBER) {
		return;
	}
}

int NurbsShape::Evaluate(double p, TrajectoryParamType ptype, TrajVertex& pos) {
	if (order_ < 1 || ctrlPoint_.size() < order_ || GetLength() < SMALL_NUMBER) {
		return -1;
	}

	double s = p;

	if (ptype == TRAJ_PARAM_TYPE_TIME) {
		pline_.Time2S(p, s);
	}

	pline_.Evaluate(s, pos, pline_.vIndex_);

	EvaluateInternal(pos.p, pos);

	return 0;
}

double NurbsShape::GetStartTime() {
	if (ctrlPoint_.size() == 0) {
		return 0.0;
	}

	return ctrlPoint_[0].time_;
}

double NurbsShape::GetDuration() {
	if (ctrlPoint_.size() == 0) {
		return 0.0;
	}

	return ctrlPoint_.back().time_ - ctrlPoint_[0].time_;
}
