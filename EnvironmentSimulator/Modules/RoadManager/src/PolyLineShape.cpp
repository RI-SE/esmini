#include "PolyLineShape.hpp"

void PolyLineShape::AddVertex(Position pos, double time, bool calculateHeading) {
	Vertex* v = new Vertex();
	v->pos_ = pos;
	vertex_.push_back(v);
	pline_.AddVertex({pos.GetTrajectoryS(), pos.GetX(), pos.GetY(), pos.GetZ(), pos.GetH(), time, 0.0, 0.0,
					  calculateHeading});
}

int PolyLineShape::Evaluate(double p, TrajectoryParamType ptype, TrajVertex& pos) {
	double s = 0;
	int i = 0;

	if (pline_.GetNumberOfVertices() < 1) {
		return -1;
	}

	if (ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_S && p > pline_.GetVertex(-1)->s
		|| ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_TIME && p > pline_.GetVertex(-1)->time) {
		// end of trajectory
		s = GetLength();
		i = (int)vertex_.size() - 1;
	} else {
		for (; i < vertex_.size() - 1
			   && (ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_S && pline_.vertex_[i + 1].s < p
				   || ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_TIME && pline_.vertex_[i + 1].time < p);
			 i++)
			;

		if (ptype == TrajectoryParamType::TRAJ_PARAM_TYPE_TIME) {
			double a = (p - pline_.vertex_[i].time)
					   / (pline_.vertex_[i + 1].time - pline_.vertex_[i].time);	 // a = interpolation factor
			s = pline_.vertex_[i].s + a * (pline_.vertex_[i + 1].s - pline_.vertex_[i].s);
		} else {
			s = p;
		}
	}

	pline_.Evaluate(s, pos, i);

	return 0;
}

double PolyLineShape::GetStartTime() {
	if (vertex_.size() == 0) {
		return 0.0;
	}

	return pline_.vertex_[0].time;
}

double PolyLineShape::GetDuration() {
	if (vertex_.size() == 0) {
		return 0.0;
	}

	return pline_.vertex_.back().time - pline_.vertex_[0].time;
}
