#include "RMTrajectory.hpp"

void RMTrajectory::Freeze() {
	if (shape_->type_ == Shape::ShapeType::POLYLINE) {
		PolyLineShape* pline = (PolyLineShape*)shape_;

		for (size_t i = 0; i < pline->vertex_.size(); i++) {
			Position* pos = &pline->vertex_[i]->pos_;
			pos->ReleaseRelation();

			if (pline->pline_.vertex_[i].calcHeading) {
				pline->pline_.UpdateVertex((int)i, pos->GetX(), pos->GetY(), pos->GetZ());
			} else {
				pline->pline_.UpdateVertex((int)i, pos->GetX(), pos->GetY(), pos->GetZ(), pos->GetH());
			}
		}
	} else if (shape_->type_ == Shape::ShapeType::CLOTHOID) {
		ClothoidShape* clothoid = (ClothoidShape*)shape_;

		clothoid->pos_.ReleaseRelation();

		clothoid->spiral_->SetX(clothoid->pos_.GetX());
		clothoid->spiral_->SetY(clothoid->pos_.GetY());
		clothoid->spiral_->SetHdg(clothoid->pos_.GetH());

		clothoid->CalculatePolyLine();
	} else {
		NurbsShape* nurbs = (NurbsShape*)shape_;

		nurbs->CalculatePolyLine();
	}
}

double RMTrajectory::GetTimeAtS(double s) {
	// Find out corresponding time-value using polyline representation
	TrajVertex v;
	shape_->pline_.Evaluate(s, v);

	return v.time;
}

double RMTrajectory::GetStartTime() {
	return shape_->GetStartTime();
}

double RMTrajectory::GetDuration() {
	return shape_->GetDuration();
}

static double GetMaxSegmentLen(Position* pos,
							   double min,
							   double max,
							   double pitchResScale,
							   double rollResScale) {
	double max_segment_length;

	// Consider rate of change of pitch and roll for segment length to influence
	// the tesselation (triangulation) of road surface model

	double zRoadPrimPrim = pos->GetZRoadPrimPrim();
	double roadSuperElevationPrim = pos->GetRoadSuperElevationPrim();
	double max_segment_length_candidate1 = pitchResScale / MAX(SMALL_NUMBER, abs(zRoadPrimPrim));
	double max_segment_length_candidate2 = rollResScale / MAX(SMALL_NUMBER, abs(roadSuperElevationPrim));

	max_segment_length = MIN(max_segment_length_candidate1, max_segment_length_candidate2);

	// Adjust for slope
	max_segment_length = max_segment_length / sqrt(pow(pos->GetZRoadPrim(), 2) + 1);

	max_segment_length = MAX(min, MIN(max, max_segment_length));

	return max_segment_length;
}
	