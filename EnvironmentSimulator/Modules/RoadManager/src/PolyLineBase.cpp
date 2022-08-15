#include "PolyLineBase.hpp"

int PolyLineBase::EvaluateSegmentByLocalS(int i, double local_s, double cornerRadius, TrajVertex& pos) {
	TrajVertex* vp0 = &vertex_[i];

	if (i >= GetNumberOfVertices() - 1) {
		pos.x = vp0->x;
		pos.y = vp0->y;
		pos.z = vp0->z;
		pos.h = vp0->h;
		pos.s = vp0->s;
		pos.p = vp0->p;
		pos.time = vp0->time;
		pos.speed = vp0->speed;
	} else if (i >= 0) {
		TrajVertex* vp1 = &vertex_[i + 1];

		double length = MAX(vertex_[i + 1].s - vertex_[i].s, SMALL_NUMBER);

		local_s = CLAMP(local_s, 0, length);

		double a = local_s / length;  // a = interpolation factor

		pos.x = (1 - a) * vp0->x + a * vp1->x;
		pos.y = (1 - a) * vp0->y + a * vp1->y;
		pos.z = (1 - a) * vp0->z + a * vp1->z;
		pos.time = (1 - a) * vp0->time + a * vp1->time;
		pos.speed = (1 - a) * vp0->speed + a * vp1->speed;
		pos.s = (1 - a) * vp0->s + a * vp1->s;
		pos.p = (1 - a) * vp0->p + a * vp1->p;

		if (vertex_[i + 1].calcHeading && !interpolateHeading_) {
			// Strategy: Align to line, but interpolate at corners
			double radius = MIN(4.0, length);
			if (local_s < radius) {
				// passed a corner
				a = (radius + local_s) / (2 * radius);
				if (i > 0) {
					pos.h = GetAngleInInterval2PI(vertex_[i - 1].h
												  + a * GetAngleDifference(vertex_[i].h, vertex_[i - 1].h));
				} else {
					// No previous value to interpolate
					pos.h = vertex_[i].h;
				}
			} else if (local_s > length - radius) {
				a = (radius + (length - local_s)) / (2 * radius);
				if (i > GetNumberOfVertices() - 2) {
					// Last segment, no next point to interpolate
					pos.h = a * vertex_[i].h;
				} else {
					pos.h = GetAngleInInterval2PI(
						vertex_[i].h + (1 - a) * GetAngleDifference(vertex_[i + 1].h, vertex_[i].h));
				}
			} else {
				pos.h = vertex_[i].h;
			}
		} else {
			// Interpolate
			pos.h = GetAngleInInterval2PI(vp0->h + a * GetAngleDifference(vp1->h, vp0->h));
		}
	} else {
		return -1;
	}

	return 0;
}

TrajVertex* PolyLineBase::AddVertex(double x, double y, double z, double h) {
	TrajVertex v;

	v.calcHeading = false;
	vertex_.push_back(v);

	return UpdateVertex(GetNumberOfVertices() - 1, x, y, z, GetAngleInInterval2PI(h));
}

TrajVertex* PolyLineBase::AddVertex(double x, double y, double z) {
	TrajVertex v;

	v.calcHeading = true;
	vertex_.push_back(v);

	return UpdateVertex(GetNumberOfVertices() - 1, x, y, z);
}

TrajVertex* PolyLineBase::AddVertex(TrajVertex p) {
	vertex_.push_back(p);

	if (p.calcHeading) {
		return UpdateVertex(GetNumberOfVertices() - 1, p.x, p.y, p.z);
	} else {
		return UpdateVertex(GetNumberOfVertices() - 1, p.x, p.y, p.z, p.h);
	}
}

TrajVertex* PolyLineBase::UpdateVertex(int i, double x, double y, double z) {
	TrajVertex* v = &vertex_[i];

	v->x = x;
	v->y = y;
	v->z = z;

	if (i > 0) {
		TrajVertex* vp = &vertex_[i - 1];

		if (v->calcHeading) {
			// Calulate heading from line segment between this and previous vertices
			if (PointDistance2D(v->x, v->y, vp->x, v->y) < SMALL_NUMBER) {
				// If points conside, use heading of previous vertex
				v->h = vp->h;
			} else {
				v->h = GetAngleInInterval2PI(atan2(v->y - vp->y, v->x - vp->x));
			}
		}

		if (vp->calcHeading) {
			// Update heading of previous vertex now that outgoing line segment is known
			vp->h = v->h;
		}

		// Update polyline length
		double dist = PointDistance2D(x, y, vp->x, vp->y);
		length_ += dist;
	} else if (i == 0) {
		length_ = 0;
	}

	v->s = length_;

	return &vertex_[i];
}

TrajVertex* PolyLineBase::UpdateVertex(int i, double x, double y, double z, double h) {
	TrajVertex* v = &vertex_[i];

	v->h = h;

	UpdateVertex(i, x, y, z);

	return &vertex_[i];
}

int PolyLineBase::Evaluate(double s, TrajVertex& pos, double cornerRadius, int startAtIndex) {
	double s_local = 0;
	int i = startAtIndex;

	if (GetNumberOfVertices() < 1) {
		return -1;
	}

	if (s > GetVertex(-1)->s) {
		// end of trajectory
		s = length_;
		s_local = 0;
		i = GetNumberOfVertices() - 1;
	} else {
		for (; i < GetNumberOfVertices() - 1 && vertex_[i + 1].s <= s; i++)
			;

		double s0 = vertex_[i].s;
		s_local = s - s0;
	}

	EvaluateSegmentByLocalS(i, s_local, cornerRadius, pos);
	pos.s = s;

	return i;
}

int PolyLineBase::Evaluate(double s, TrajVertex& pos, double cornerRadius) {
	return Evaluate(s, pos, cornerRadius, 0);
}

int PolyLineBase::Evaluate(double s, TrajVertex& pos, int startAtIndex) {
	return Evaluate(s, pos, 0.0, startAtIndex);
}

int PolyLineBase::Evaluate(double s, TrajVertex& pos) {
	return Evaluate(s, pos, 0.0, 0);
}

int PolyLineBase::Time2S(double time, double& s) {
	if (GetNumberOfVertices() < 1) {
		return -1;
	}

	// start looking from current index
	int i = vIndex_;

	for (size_t j = 0; j < GetNumberOfVertices(); j++) {
		if (vertex_[i].time <= time && vertex_[i + 1].time > time) {
			double w = (time - vertex_[i].time) / (vertex_[i + 1].time - vertex_[i].time);
			s = vertex_[i].s + w * (vertex_[i + 1].s - vertex_[i].s);
			vIndex_ = i;
			return 0;
		}

		if (++i >= GetNumberOfVertices() - 1) {
			// Reached end of buffer, continue from start
			i = 0;
		}
	}

	// s seems out of range, grab last element
	s = GetVertex(-1)->s;

	return 0;
}

int PolyLineBase::FindClosestPoint(double xin, double yin, TrajVertex& pos, int& index, int startAtIndex) {
	// look along the line segments
	TrajVertex tmpPos;
	double sLocal = 0.0;
	double sLocalMin = 0.0;
	int iMin = startAtIndex;
	double distMin = LARGE_NUMBER;

	// If a teleportation is made by the Ghost, a reset of trajectory has benn made. Hence, we can't look from
	// the usual point ad has to set startAtIndex = 0

	if (startAtIndex > GetNumberOfVertices() - 1) {
		startAtIndex = 0;
		index = 0;
	}

	// Find closest line segment
	for (int i = startAtIndex; i < GetNumberOfVertices() - 1; i++) {
		ProjectPointOnVector2D(xin, yin, vertex_[i].x, vertex_[i].y, vertex_[i + 1].x, vertex_[i + 1].y,
							   tmpPos.x, tmpPos.y);
		double distTmp = PointDistance2D(xin, yin, tmpPos.x, tmpPos.y);

		bool inside = PointInBetweenVectorEndpoints(tmpPos.x, tmpPos.y, vertex_[i].x, vertex_[i].y,
													vertex_[i + 1].x, vertex_[i + 1].y, sLocal);
		if (!inside) {
			// Find combined longitudinal and lateral distance to line endpoint
			// sLocal represent now (outside line segment) distance to closest line segment end point
			distTmp = sqrt(distTmp * distTmp + sLocal * sLocal);
			if (sLocal < 0) {
				sLocal = 0;
			} else {
				sLocal = vertex_[i + 1].s - vertex_[i].s;
			}
		} else {
			// rescale normalized s
			sLocal *= (vertex_[i + 1].s - vertex_[i].s);
		}

		if (distTmp < distMin) {
			iMin = (int)i;
			sLocalMin = sLocal;
			distMin = distTmp;
		}
	}

	if (distMin < LARGE_NUMBER) {
		EvaluateSegmentByLocalS(iMin, sLocalMin, 0.0, pos);
		index = iMin;
		return 0;
	} else {
		return -1;
	}
}

int PolyLineBase::FindPointAhead(double s_start,
								 double distance,
								 TrajVertex& pos,
								 int& index,
								 int startAtIndex) {
	index = Evaluate(s_start + distance, pos, startAtIndex);

	return 0;
}

TrajVertex* PolyLineBase::GetVertex(int index) {
	if (GetNumberOfVertices() < 1) {
		return nullptr;
	}

	if (index == -1) {
		return &vertex_.back();
	} else {
		return &vertex_[index];
	}
}

void PolyLineBase::Reset() {
	vertex_.clear();
	vIndex_ = 0;
	length_ = 0;
}
