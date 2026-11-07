#include "Ball/BallHandCollisionSolver.h"
#include "Ball/BallCollisionSolver.h"
#include "Ball/BallPalmGeometry.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

const FBallMotionSegment *FBallHandCollisionSolver::SegmentAt(const FBallMotionFrame &F, const FBallColliderMotion &C,
															  double T)
{
	// At an interior knot use outgoing motion; the final boundary uses the final segment.
	for (int32 I = C.SegmentCount - 1; I >= 0; --I)
	{
		const auto &S = F.Segments[C.FirstSegment + I];
		if (T >= S.StartFraction - 1.e-12 && T <= S.EndFraction + 1.e-12)
		{
			return &S;
		}
	}
	return nullptr;
}
FVector FBallHandCollisionSolver::CenterAt(const FBallMotionSegment &S, double T)
{
	return FMath::Lerp(S.Start, S.End,
					   FMath::Clamp((T - S.StartFraction) / (S.EndFraction - S.StartFraction), 0.0, 1.0));
}
double FBallHandCollisionSolver::Separation(const FBallMotionFrame &F, const FBallColliderMotion &C, double T,
											const FVector &Ball, double Radius)
{
	const auto *S = SegmentAt(F, C, T);
	if (!S)
	{
		return TNumericLimits<double>::Max();
	}
	if (C.Shape == EBallMotionShape::RoundedBox)
		return FBallPalmGeometry::Distance(Ball, Radius, FBallPalmGeometry::CenterAt(C, *S, T),
										   FBallPalmGeometry::OrientationAt(*S, T), C.HalfExtents, C.RoundRadius)
			.Separation;
	const FVector P = C.Orientation.UnrotateVector(Ball - CenterAt(*S, T));
	if (C.Shape == EBallMotionShape::Sphere)
	{
		return P.Size() - C.Radius - Radius;
	}
	const FVector Outside = P.GetAbs() - C.HalfExtents;
	const FVector Positive(FMath::Max(Outside.X, 0.0), FMath::Max(Outside.Y, 0.0), FMath::Max(Outside.Z, 0.0));
	return Positive.Size() + FMath::Min(Outside.GetMax(), 0.0) - Radius;
}
EBallQueryResult FBallHandCollisionSolver::Query(const FBallMotionFrame &F, const FBallColliderMotion &C,
												 const FBallCollisionQuery &Q, FBallContact &Contact)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Handball_RelativeMotionCCD);
	const double Begin = Q.Time.Frame == Q.StepFrame ? Q.Time.Fraction : 1;
	const auto *S = SegmentAt(F, C, Begin);
	if (!S)
	{
		return EBallQueryResult::Clear;
	}
	const double End = Q.bProximity ? Begin : Q.EndFraction;
	// At an expired lifetime endpoint there is no outgoing sweep. SegmentAt deliberately
	// keeps that endpoint for proximity probes, but must not extend it to a later interval.
	if (!Q.bProximity && Begin >= S->EndFraction - 1.e-12)
		return EBallQueryResult::Clear;
	// Caller splits drift at every trajectory knot/lifetime edge. Never replace a curved path with its full-step chord.
	if (!Q.bProximity && End > S->EndFraction + 1.e-10)
	{
		return EBallQueryResult::InvalidQuery;
	}
	if (C.Shape == EBallMotionShape::RoundedBox)
	{
		const auto Result = FBallPalmGeometry::Sweep(F, C, *S, Q, Contact);
		if (Result != EBallQueryResult::Contact)
			return Result;
		Contact.Collider = C.Id;
		Contact.Category = C.Category;
		Contact.Material = C.Material;
		if (C.FaceMaterials.bEnabled)
		{
			const FVector N = FBallPalmGeometry::OrientationAt(*S, FMath::Lerp(Begin, End, Contact.TimeOfImpact))
								  .UnrotateVector(Contact.Normal);
			Contact.Material = FMath::Abs(N.Z) >= FMath::Max(FMath::Abs(N.X), FMath::Abs(N.Y))
								   ? (N.Z >= 0 ? C.FaceMaterials.Front : C.FaceMaterials.Back)
								   : C.FaceMaterials.Side;
		}
		Contact.Participant = C.Participant;
		Contact.OriginRevision = C.OriginRevision;
		Contact.HandSide = uint8(C.Side);
		Contact.FirstSample = S->FirstSample;
		Contact.SecondSample = S->SecondSample;
		return Result;
	}
	const FVector H0 = CenterAt(*S, Begin), H1 = CenterAt(*S, End);
	FBallCollisionQuery Relative = Q;
	Relative.Start = C.Orientation.UnrotateVector(Q.Start - H0);
	Relative.End = C.Orientation.UnrotateVector(Q.End - H1);
	FBallContact Hit;
	const bool bHit = C.Shape == EBallMotionShape::Sphere
						  ? FBallCollisionSolver::SweepSphere(Relative, FVector::ZeroVector, C.Radius, Hit)
						  : FBallCollisionSolver::SweepBox(Relative, -C.HalfExtents, C.HalfExtents, Hit);
	if (!bHit)
	{
		return EBallQueryResult::Clear;
	}
	const double Impact = FMath::Lerp(Begin, End, Hit.TimeOfImpact);
	const FVector Center = CenterAt(*S, Impact);
	Contact = Hit;
	Contact.Collider = C.Id;
	Contact.Category = C.Category;
	Contact.Material = C.Material;
	Contact.Normal = C.Orientation.RotateVector(Hit.Normal);
	Contact.Position = Center + C.Orientation.RotateVector(Hit.Position);
	Contact.SurfaceVelocity = (S->End - S->Start) / ((S->EndFraction - S->StartFraction) * F.DeltaTime) +
							  FVector::CrossProduct(S->SurfaceAngularVelocity, Contact.Position - Center);
	Contact.Participant = C.Participant;
	Contact.OriginRevision = C.OriginRevision;
	Contact.HandSide = uint8(C.Side);
	Contact.FirstSample = S->FirstSample;
	Contact.SecondSample = S->SecondSample;
	return EBallQueryResult::Contact;
}
double FBallHandCollisionSolver::MaximumRelativeTravel(const FBallMotionFrame &F, const FBallCollisionQuery &Q)
{
	const FVector Ball = Q.End - Q.Start;
	double Maximum = Ball.Size();
	for (int32 I = 0; I < F.ColliderCount; ++I)
	{
		const auto *S = SegmentAt(F, F.Colliders[I], Q.Time.Fraction);
		if (!S)
		{
			continue;
		}
		Maximum = FMath::Max(Maximum, (Ball - (CenterAt(*S, Q.EndFraction) - CenterAt(*S, Q.Time.Fraction))).Size());
		const auto &C = F.Colliders[I];
		if (C.Shape == EBallMotionShape::RoundedBox)
			Maximum = FMath::Max(Maximum, Ball.Size() + (S->End - S->Start).Size() +
											  FBallPalmGeometry::AngularDisplacement(*S).Size() *
												  (C.HalfExtents.Size() + 2 * C.PivotToCenter.Size()));
	}
	return Maximum;
}
double FBallHandCollisionSolver::NextBoundary(const FBallMotionFrame &F, double T, double End)
{
	double Next = End;
	for (int32 I = 0; I < F.SegmentCount; ++I)
	{
		const auto &S = F.Segments[I];
		if (S.StartFraction > T + 1.e-11)
		{
			Next = FMath::Min(Next, S.StartFraction);
		}
		if (S.EndFraction > T + 1.e-11)
		{
			Next = FMath::Min(Next, S.EndFraction);
		}
	}
	return Next;
}
bool FBallHandCollisionSolver::CouldReach(const FBallMotionFrame &F, const FVector &Ball, double Radius)
{
	for (int32 I = 0; I < F.ColliderCount; ++I)
	{
		const auto &C = F.Colliders[I];
		const double R = Radius + (C.Shape == EBallMotionShape::Sphere ? C.Radius : C.HalfExtents.Size()) +
						 (C.Shape == EBallMotionShape::RoundedBox ? 2 * C.PivotToCenter.Size() : 0);
		for (int32 J = 0; J < C.SegmentCount; ++J)
		{
			const auto &S = F.Segments[C.FirstSegment + J];
			if (FMath::PointDistToSegmentSquared(Ball, S.Start, S.End) <= FMath::Square(R))
			{
				return true;
			}
		}
	}
	return false;
}
