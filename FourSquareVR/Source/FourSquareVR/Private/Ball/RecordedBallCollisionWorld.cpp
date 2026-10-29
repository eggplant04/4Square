#include "Ball/RecordedBallCollisionWorld.h"

FRecordedBallCollisionWorld::FRecordedBallCollisionWorld(uint32 Revision) : WorldRevision(Revision)
{
	Planes.Reserve(32);
}

bool FRecordedBallCollisionWorld::AddPlane(FBallColliderId Id, const FVector& Point, const FVector& OutwardNormal)
{
	if (!Id.IsValid() || Point.ContainsNaN() || OutwardNormal.ContainsNaN() || !OutwardNormal.IsUnit(1.e-6)
		|| Planes.Num() == 32 || WorldRevision == 0 || bSealed) { return false; }
	for (const FPlaneRecord& Plane : Planes) { if (Plane.Id == Id) { return false; } }
	Planes.Add({Id, Point, OutwardNormal});
	return true;
}

EBallQueryResult FRecordedBallCollisionWorld::SweepSphere(const FBallCollisionQuery& Query, FBallContact& OutContact) const
{
	if (WorldRevision == 0 || Query.Start.ContainsNaN() || Query.End.ContainsNaN()
		|| !FMath::IsFinite(Query.Radius) || Query.Radius <= 0 || !Query.Time.IsValid()
		|| !FMath::IsFinite(Query.EndFraction) || Query.EndFraction <= Query.Time.Fraction || Query.EndFraction > 1)
	{ return EBallQueryResult::InvalidQuery; }
	bSealed = true; // Geometry cannot change underneath already recorded inputs with the same revision.
	bool bFound = false;
	FBallContact Best;
	Best.TimeOfImpact = 2;
	for (const FPlaneRecord& Plane : Planes)
	{
		const double StartDistance = FVector::DotProduct(Query.Start - Plane.Point, Plane.Normal) - Query.Radius;
		const double EndDistance = FVector::DotProduct(Query.End - Plane.Point, Plane.Normal) - Query.Radius;
		if (!FMath::IsFinite(StartDistance) || !FMath::IsFinite(EndDistance)) { return EBallQueryResult::InvalidQuery; }
		if (StartDistance > 0 && EndDistance > 0) { continue; }
		if (StartDistance >= 0 && EndDistance >= StartDistance) { continue; }
		const double Time = StartDistance <= 0 ? 0 : StartDistance / (StartDistance - EndDistance);
		if (!bFound || Time < Best.TimeOfImpact || (Time == Best.TimeOfImpact && Plane.Id.Value < Best.Collider.Value))
		{
			bFound = true;
			Best.Collider = Plane.Id;
			Best.TimeOfImpact = Time;
			Best.Normal = Plane.Normal;
			Best.Position = FMath::Lerp(Query.Start, Query.End, Time) - Plane.Normal * Query.Radius;
			Best.PenetrationDepth = FMath::Max(0.0, -StartDistance);
		}
	}
	if (bFound) { OutContact = Best; }
	return bFound ? EBallQueryResult::Contact : EBallQueryResult::Clear;
}
