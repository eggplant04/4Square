#include "Ball/BallPalmGeometry.h"
#include "Ball/BallCollisionSolver.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

namespace
{
double Alpha(const FBallMotionSegment &S, double T)
{
	return FMath::Clamp((T - S.StartFraction) / (S.EndFraction - S.StartFraction), 0.0, 1.0);
}
FVector Pivot(const FBallColliderMotion &C, const FBallMotionSegment &S, bool End)
{
	return (End ? S.End : S.Start) - (End ? S.EndOrientation : S.StartOrientation).RotateVector(C.PivotToCenter);
}
struct FProjection
{
	double A = 0, B = 0, C = 0;
	double At(double T) const
	{
		return A * FMath::Cos(T) + B * FMath::Sin(T) + C;
	}
};
FProjection ProjectRotation(const FVector &V, const FVector &Axis, const FVector &Normal)
{
	const double Constant = FVector::DotProduct(Normal, Axis) * FVector::DotProduct(Axis, V);
	return {FVector::DotProduct(Normal, V) - Constant, FVector::DotProduct(Normal, FVector::CrossProduct(Axis, V)),
			Constant};
}
/** Exact maximum of the support of a rotating rounded box along a FIXED world normal,
 * minus relative linear translation. Each axis projection is a*cos(theta)+b*sin(theta)+c.
 * Partition at its zeros so abs(projection) has a fixed sign. On each region the support
 * is A*cos(theta)+B*sin(theta)+C+D*theta; check endpoints and every analytic derivative root.
 * At most six zero boundaries for a <pi arc. Fixed scratch, no sampled-overlap assumption. */
bool MaximumSupport(const FBallColliderMotion &C, const FQuat &Rotation, const FVector &Axis, double Angle,
					const FVector &Normal, double LinearPerRadian, double &Maximum)
{
	FProjection Axes[3];
	const FVector Core = C.HalfExtents - FVector(C.RoundRadius);
	double Boundaries[10] = {0, Angle};
	int32 Count = 2;
	for (int32 I = 0; I < 3; ++I)
	{
		FVector E = FVector::ZeroVector;
		E[I] = 1;
		Axes[I] = ProjectRotation(Rotation.RotateVector(E), Axis, Normal);
		const auto &P = Axes[I];
		const double R = FMath::Sqrt(P.A * P.A + P.B * P.B);
		if (R < 1.e-14 || FMath::Abs(P.C) > R)
			continue;
		const double Phase = FMath::Atan2(P.B, P.A), Root = FMath::Acos(FMath::Clamp(-P.C / R, -1.0, 1.0));
		for (int32 Sign : {-1, 1})
			for (int32 K = -1; K <= 1; ++K)
			{
				const double T = Phase + Sign * Root + K * 2 * UE_DOUBLE_PI;
				if (T <= 1.e-12 || T >= Angle - 1.e-12)
					continue;
				if (Count == UE_ARRAY_COUNT(Boundaries))
					return false;
				Boundaries[Count++] = T;
			}
	}
	for (int32 I = 1; I < Count; ++I)
	{
		const double T = Boundaries[I];
		int32 J = I;
		while (J > 0 && Boundaries[J - 1] > T)
		{
			Boundaries[J] = Boundaries[J - 1];
			--J;
		}
		Boundaries[J] = T;
	}
	const auto Lever = ProjectRotation(Rotation.RotateVector(C.PivotToCenter), Axis, Normal);
	Maximum = -TNumericLimits<double>::Max();
	for (int32 I = 1; I < Count; ++I)
	{
		const double Low = Boundaries[I - 1], High = Boundaries[I];
		FProjection P = Lever;
		P.C += C.RoundRadius;
		for (int32 J = 0; J < 3; ++J)
		{
			const double Weight = Core[J] * (Axes[J].At((Low + High) * .5) >= 0 ? 1 : -1);
			P.A += Weight * Axes[J].A;
			P.B += Weight * Axes[J].B;
			P.C += Weight * Axes[J].C;
		}
		const auto Check = [&](double T) { Maximum = FMath::Max(Maximum, P.At(T) + LinearPerRadian * T); };
		Check(Low);
		Check(High);
		const double R = FMath::Sqrt(P.A * P.A + P.B * P.B);
		if (R < 1.e-14 || FMath::Abs(LinearPerRadian) > R)
			continue;
		const double Phase = FMath::Atan2(-P.A, P.B), Root = FMath::Acos(FMath::Clamp(-LinearPerRadian / R, -1.0, 1.0));
		for (int32 Sign : {-1, 1})
			for (int32 K = -1; K <= 1; ++K)
			{
				const double T = Phase + Sign * Root + K * 2 * UE_DOUBLE_PI;
				if (T > Low && T < High)
					Check(T);
			}
	}
	return FMath::IsFinite(Maximum);
}
} // namespace
FQuat FBallPalmGeometry::OrientationAt(const FBallMotionSegment &S, double T)
{
	return InterpolateOrientation(S.StartOrientation, S.EndOrientation, Alpha(S, T));
}
FQuat FBallPalmGeometry::InterpolateOrientation(const FQuat &A, const FQuat &B, double T)
{
	const FVector Angle = RotationVector(A, B);
	const double Magnitude = Angle.Size();
	// Exponential-map slerp, including small angles: engine nlerp shortcuts would make the
	// assumed constant angular derivative slightly inconsistent with contact surface velocity.
	return Magnitude > 1.e-14 ? (FQuat(Angle / Magnitude, Magnitude * T) * A).GetNormalized() : A;
}
FVector FBallPalmGeometry::AngularDisplacement(const FBallMotionSegment &S)
{
	return RotationVector(S.StartOrientation, S.EndOrientation);
}
FVector FBallPalmGeometry::RotationVector(const FQuat &A, const FQuat &B)
{
	FQuat D = (B * A.Inverse()).GetNormalized();
	if (D.W < 0)
		D = D * -1;
	const FVector V(D.X, D.Y, D.Z);
	const double SinHalf = V.Size();
	// atan2 retains tiny angular deltas that acos(W) rounds to zero, and handles q == -q.
	return SinHalf > 1.e-15 ? V * (2 * FMath::Atan2(SinHalf, D.W) / SinHalf) : FVector::ZeroVector;
}
FVector FBallPalmGeometry::CenterAt(const FBallColliderMotion &C, const FBallMotionSegment &S, double T)
{
	return FMath::Lerp(Pivot(C, S, false), Pivot(C, S, true), Alpha(S, T)) +
		   OrientationAt(S, T).RotateVector(C.PivotToCenter);
}
FVector FBallPalmGeometry::SurfaceVelocity(const FBallColliderMotion &C, const FBallMotionSegment &S, double T,
										   double Dt, const FVector &Point)
{
	const double Span = (S.EndFraction - S.StartFraction) * Dt;
	const FVector P0 = Pivot(C, S, false), P1 = Pivot(C, S, true);
	const FVector Controller = FMath::Lerp(P0, P1, Alpha(S, T));
	// Equivalently Vcenter + omega x (point-center). The offset contribution occurs exactly once.
	return (P1 - P0) / Span + FVector::CrossProduct(AngularDisplacement(S) / Span, Point - Controller);
}
FBallPalmDistance FBallPalmGeometry::Distance(const FVector &Ball, double BallRadius, const FVector &Center,
											  const FQuat &Orientation, const FVector &HalfExtents, double RoundRadius)
{
	FBallPalmDistance D;
	const FVector P = Orientation.UnrotateVector(Ball - Center), Core = HalfExtents - FVector(RoundRadius);
	FVector Closest, N;
	for (int32 I = 0; I < 3; ++I)
		Closest[I] = FMath::Clamp(P[I], -Core[I], Core[I]);
	N = P - Closest;
	const double Length = N.Size();
	if (Length > 1.e-12)
	{
		N /= Length;
		D.Separation = Length - RoundRadius - BallRadius;
	}
	else
	{
		// Inside the core: nearest face, with deterministic X/Y/Z tie order.
		const FVector Gap = Core - P.GetAbs();
		int32 Axis = 0;
		if (Gap.Y < Gap[Axis])
			Axis = 1;
		if (Gap.Z < Gap[Axis])
			Axis = 2;
		N = FVector::ZeroVector;
		N[Axis] = P[Axis] >= 0 ? 1 : -1;
		Closest = P;
		Closest[Axis] = N[Axis] * Core[Axis];
		D.Separation = -Gap[Axis] - RoundRadius - BallRadius;
	}
	uint32 Multiplier = 1;
	for (int32 I = 0; I < 3; ++I)
	{
		D.Feature += (N[I] < -1.e-10 ? 0u : N[I] > 1.e-10 ? 2u : 1u) * Multiplier;
		Multiplier *= 3;
	}
	D.Normal = Orientation.RotateVector(N);
	D.Point = Center + Orientation.RotateVector(Closest + N * RoundRadius);
	return D;
}
EBallQueryResult FBallPalmGeometry::Sweep(const FBallMotionFrame &F, const FBallColliderMotion &C,
										  const FBallMotionSegment &S, const FBallCollisionQuery &Q,
										  FBallContact &Contact)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Handball_RotatingRoundedBoxCCD);
	const double Begin = Q.Time.Frame == Q.StepFrame ? Q.Time.Fraction : 1;
	const double End = Q.bProximity ? Begin : Q.EndFraction;
	auto *Stats = Q.MotionStatistics;
	if (Stats)
		++Stats->Queries;
	const auto Evaluate = [&](double T) {
		if (Stats)
			++Stats->DistanceEvaluations;
		const double Fraction = FMath::Lerp(Begin, End, T);
		return Distance(FMath::Lerp(Q.Start, Q.End, T), Q.Radius, CenterAt(C, S, Fraction), OrientationAt(S, Fraction),
						C.HalfExtents, C.RoundRadius);
	};
	const auto Report = [&](double T, const FBallPalmDistance &D) {
		Contact.TimeOfImpact = T;
		Contact.Position = D.Point;
		Contact.Normal = D.Normal;
		Contact.FeatureId = D.Feature;
		Contact.Separation = D.Separation;
		Contact.PenetrationDepth = FMath::Max(0.0, -D.Separation);
		Contact.SurfaceVelocity = SurfaceVelocity(C, S, FMath::Lerp(Begin, End, T), F.DeltaTime, D.Point);
	};
	// Rotation-invariant enclosing sphere about the tracked pivot, swept in relative translation.
	// This only culls candidates; it never supplies a normal or an impact.
	const FVector P0 = FMath::Lerp(Pivot(C, S, false), Pivot(C, S, true), Alpha(S, Begin));
	const FVector P1 = FMath::Lerp(Pivot(C, S, false), Pivot(C, S, true), Alpha(S, End));
	const double Bound = Q.Radius + C.PivotToCenter.Size() + (C.HalfExtents - FVector(C.RoundRadius)).Size() +
						 C.RoundRadius + (Q.bProximity ? Q.ProbeDistance : C.CCDDistanceTolerance);
	if (FMath::PointDistToSegmentSquared(FVector::ZeroVector, Q.Start - P0, Q.End - P1) > Bound * Bound)
		return EBallQueryResult::Clear;
	const auto Initial = Evaluate(0);
	if (Q.bProximity)
	{
		if (Initial.Separation > Q.ProbeDistance)
			return EBallQueryResult::Clear;
		Report(0, Initial);
		return EBallQueryResult::Contact;
	}
	const double Duration = (End - Begin) * F.DeltaTime;
	if (Duration <= 0)
		return EBallQueryResult::InvalidQuery;
	const FVector PivotDelta =
		(Pivot(C, S, true) - Pivot(C, S, false)) * ((End - Begin) / (S.EndFraction - S.StartFraction));
	const FVector RelativeDelta = Q.End - Q.Start - PivotDelta;
	const FVector Angle = AngularDisplacement(S) * ((End - Begin) / (S.EndFraction - S.StartFraction));
	const double PivotRadius = C.PivotToCenter.Size() + (C.HalfExtents - FVector(C.RoundRadius)).Size();
	const double GlobalTravel = RelativeDelta.Size() + Angle.Size() * PivotRadius;
	const double Epsilon = C.CCDDistanceTolerance;
	// No rotation: exact existing sphere/core-box sweep, radius enlarged by rounding.
	if (Angle.IsNearlyZero(1.e-12))
	{
		const FQuat Rotation = OrientationAt(S, Begin);
		FBallCollisionQuery Local = Q;
		Local.Start = Rotation.UnrotateVector(Q.Start - CenterAt(C, S, Begin));
		Local.End = Rotation.UnrotateVector(Q.End - CenterAt(C, S, End));
		Local.Radius += C.RoundRadius;
		FBallContact Hit;
		const FVector Core = C.HalfExtents - FVector(C.RoundRadius);
		if (!FBallCollisionSolver::SweepBox(Local, -Core, Core, Hit))
			return EBallQueryResult::Clear;
		Report(Hit.TimeOfImpact, Evaluate(Hit.TimeOfImpact));
		return EBallQueryResult::Contact;
	}
	double T = 0;
	for (int32 I = 0; I < C.MaximumCCDIterations; ++I)
	{
		if (Stats)
		{
			++Stats->Iterations;
			Stats->MaximumIterations = FMath::Max(Stats->MaximumIterations, uint32(I + 1));
		}
		const auto D = I == 0 ? Initial : Evaluate(T);
		const FVector Surface = SurfaceVelocity(C, S, FMath::Lerp(Begin, End, T), F.DeltaTime, D.Point);
		const double Closing = FVector::DotProduct((Q.End - Q.Start) / Duration - Surface, D.Normal);
		if (D.Separation < -Q.SpatialTolerance || (D.Separation <= Epsilon && Closing < -1.e-7))
		{
			Report(T, D);
			return EBallQueryResult::Contact;
		}
		// A fixed separating plane at this closest normal is valid for the entire convex body.
		// |(omega x n).r| <= |omega x n| * maximum radius about the pivot.
		// Thus Mu bounds projected closing motion over the REMAINING interval, including all rotation.
		const double Mu = FMath::Max(0.0, -FVector::DotProduct(RelativeDelta, D.Normal) +
											  FVector::CrossProduct(Angle, D.Normal).Size() * PivotRadius);
		if (Mu <= 1.e-14 || D.Separation > Mu * (1 - T) + Epsilon)
			return EBallQueryResult::Clear;
		if (I >= 12 && I % 12 == 0 && Angle.Size() > 1.e-12)
		{
			const FVector RelativeNow = FMath::Lerp(Q.Start, Q.End, T) - FMath::Lerp(P0, P1, T);
			const double Roundoff = 1.e-10 * (1 + C.HalfExtents.Size() + C.PivotToCenter.Size() + RelativeNow.Size());
			// The current closest normal can be poor near an extremal corner graze. The radial
			// pivot-to-ball plane is another valid candidate; neither changes the contact normal.
			for (const FVector Plane : {D.Normal, RelativeNow.GetSafeNormal()})
			{
				if (Stats)
					++Stats->SupportBounds;
				const double Linear = -FVector::DotProduct(RelativeDelta, Plane) / Angle.Size();
				double Support;
				if (MaximumSupport(C, OrientationAt(S, FMath::Lerp(Begin, End, T)), Angle / Angle.Size(),
								   Angle.Size() * (1 - T), Plane, Linear, Support) &&
					FVector::DotProduct(RelativeNow, Plane) - Q.Radius - Support > Epsilon + Roundoff)
					return EBallQueryResult::Clear;
			}
		}
		double Advance = .9 * FMath::Max(0.0, D.Separation) / Mu;
		// Tangent/initial touching can have zero distance without a closing impulse. Permit only
		// an Epsilon-bounded surface excursion; no arbitrary temporal sampling or large minimum step.
		if (Advance < Epsilon / FMath::Max(GlobalTravel, 1.e-14))
			Advance = Epsilon / FMath::Max(GlobalTravel, 1.e-14);
		if (!FMath::IsFinite(Advance) || Advance <= 0 || T + Advance == T)
			break;
		if (T + Advance > 1)
		{
			// The last conservative step certifies the rest, except a <= Epsilon endpoint shell.
			const auto Last = Evaluate(1);
			if (Last.Separation <= Epsilon &&
				FVector::DotProduct((Q.End - Q.Start) / Duration - SurfaceVelocity(C, S, End, F.DeltaTime, Last.Point),
									Last.Normal) < -1.e-7)
			{
				Report(1, Last);
				return EBallQueryResult::Contact;
			}
			return EBallQueryResult::Clear;
		}
		T += Advance;
	}
	if (Stats)
		++Stats->Unresolved;
	return EBallQueryResult::Unresolved;
}
