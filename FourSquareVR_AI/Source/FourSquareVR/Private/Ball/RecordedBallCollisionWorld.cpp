#include "Ball/RecordedBallCollisionWorld.h"
#include "Ball/BallCollisionSolver.h"

FRecordedBallCollisionWorld::FRecordedBallCollisionWorld(uint32 Revision) : WorldRevision(Revision)
{
	Shapes.Reserve(32);
}

bool FRecordedBallCollisionWorld::Add(const FShapeRecord& Record)
{
	if (!Record.Id.IsValid() || !Record.Material.IsValid() || Shapes.Num() == 32 || WorldRevision == 0 || bSealed)
	{
		return false;
	}
	for (const auto& Shape : Shapes)
	{
		if (Shape.Id == Record.Id)
		{
			return false;
		}
	}
	Shapes.Add(Record);
	return true;
}

bool FRecordedBallCollisionWorld::AddPlane(FBallColliderId Id, const FVector& Point, const FVector& Normal,
										   FBallSurfaceMaterial Material)
{
	return !Point.ContainsNaN() && !Normal.ContainsNaN() && Normal.IsUnit(1.e-6) &&
		   Add({Id, Point, Normal, 0, 0, Material});
}

bool FRecordedBallCollisionWorld::AddSphere(FBallColliderId Id, const FVector& Center, double Radius,
											FBallSurfaceMaterial Material)
{
	return !Center.ContainsNaN() && FMath::IsFinite(Radius) && Radius > 0 &&
		   Add({Id, Center, FVector::ZeroVector, Radius, 1, Material});
}

bool FRecordedBallCollisionWorld::AddBox(FBallColliderId Id, const FVector& Minimum, const FVector& Maximum,
										 FBallSurfaceMaterial Material)
{
	return !Minimum.ContainsNaN() && !Maximum.ContainsNaN() && Minimum.X < Maximum.X && Minimum.Y < Maximum.Y &&
		   Minimum.Z < Maximum.Z && Add({Id, Minimum, Maximum, 0, 2, Material});
}

EBallQueryResult FRecordedBallCollisionWorld::SweepSphere(const FBallCollisionQuery& Query,
														  FBallContact& OutContact) const
{
	FBallContactBatch Batch;
	const auto Result = GatherContacts(Query, Batch);
	if (Result == EBallQueryResult::Contact)
	{
		OutContact = Batch.Contacts[0];
	}
	return Result;
}

EBallQueryResult FRecordedBallCollisionWorld::GatherContacts(const FBallCollisionQuery& Q,
															 FBallContactBatch& Output) const
{
	Output.Count = 0;
	if (WorldRevision == 0 || Q.Start.ContainsNaN() || Q.End.ContainsNaN() || !FMath::IsFinite(Q.Radius) ||
		Q.Radius <= 0 || !Q.Time.IsValid() || !FMath::IsFinite(Q.EndFraction) || Q.EndFraction <= Q.Time.Fraction ||
		Q.EndFraction > 1 || !FMath::IsFinite(Q.ProbeDistance) || Q.ProbeDistance < 0 ||
		!FMath::IsFinite(Q.SpatialTolerance) || Q.SpatialTolerance < 0)
	{
		return EBallQueryResult::InvalidQuery;
	}
	bSealed = true;
	for (const auto& Shape : Shapes)
	{
		FBallContact C;
		C.Collider = Shape.Id;
		C.Material = Shape.Material;
		bool bHit = false;
		if (Shape.Shape == 0)
		{
			C.Normal = Shape.B;
			const double Start = FVector::DotProduct(Q.Start - Shape.A, C.Normal) - Q.Radius;
			const double End = FVector::DotProduct(Q.End - Shape.A, C.Normal) - Q.Radius;
			C.Separation = Start;
			if (Q.bProximity)
			{
				bHit = Start <= Q.ProbeDistance;
			}
			else if (Start < -Q.SpatialTolerance || (Start <= Q.SpatialTolerance && End < Start - 1.e-10))
			{
				bHit = true;
			}
			else if (Start > 0 && End <= 0)
			{
				bHit = true;
				C.TimeOfImpact = Start / (Start - End);
				C.Separation = 0;
			}
			C.PenetrationDepth = C.TimeOfImpact == 0 ? FMath::Max(0.0, -Start) : 0;
			C.Position = FMath::Lerp(Q.Start, Q.End, C.TimeOfImpact) - C.Normal * (Q.Radius + C.Separation);
		}
		else if (Shape.Shape == 1)
		{
			bHit = FBallCollisionSolver::SweepSphere(Q, Shape.A, Shape.Radius, C);
		}
		else
		{
			bHit = FBallCollisionSolver::SweepBox(Q, Shape.A, Shape.B, C);
		}
		if (bHit && !Output.Add(C, Q))
		{
			return EBallQueryResult::CapacityExceeded;
		}
	}
	Output.Sort();
	return Output.Count > 0 ? EBallQueryResult::Contact : EBallQueryResult::Clear;
}
