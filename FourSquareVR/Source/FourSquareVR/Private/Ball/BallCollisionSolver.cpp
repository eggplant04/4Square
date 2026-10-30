#include "Ball/BallCollisionSolver.h"

namespace
{
double Restitution(const FBallContact &C, const FBallPhysicsConfig &P)
{
	return C.Material.bOverride ? FMath::Sqrt(P.Restitution * C.Material.Restitution) : P.Restitution;
}
double Friction(const FBallContact &C, const FBallPhysicsConfig &P)
{
	return C.Material.bOverride ? FMath::Sqrt(P.SurfaceFriction * C.Material.Friction) : P.SurfaceFriction;
}
int32 Cache(FBallState &S, const FBallContact &C, bool &bNew)
{
	bNew = false;
	for (int32 I = 0; I < S.ContactCount; ++I)
	{
		if (S.Contacts[I].Collider == C.Collider && S.Contacts[I].FeatureId == C.FeatureId)
		{
			return I;
		}
	}
	int32 Index = S.ContactCount;
	if (Index == BallLimits::Contacts)
	{
		Index = INDEX_NONE;
		for (int32 I = 0; I < S.ContactCount; ++I)
		{
			if (!S.Contacts[I].bTouching &&
				(Index == INDEX_NONE || S.Contacts[I].LastImpactFrame < S.Contacts[Index].LastImpactFrame))
			{
				Index = I;
			}
		}
		if (Index == INDEX_NONE)
		{
			return INDEX_NONE;
		}
	}
	else
	{
		++S.ContactCount;
	}
	S.Contacts[Index] = {};
	S.Contacts[Index].Collider = C.Collider;
	S.Contacts[Index].FeatureId = C.FeatureId;
	S.Contacts[Index].Episode = 1;
	bNew = true;
	return Index;
}
double Energy(const FBallState &S, const FBallPhysicsConfig &P)
{
	return 0.5 *
		   (P.Mass * S.LinearVelocity.SizeSquared() + P.GetSolidSphereInertia() * S.AngularVelocity.SizeSquared());
}
bool InitialContact(const FBallCollisionQuery &Q, FBallContact &C)
{
	C.TimeOfImpact = 0;
	C.PenetrationDepth = FMath::Max(0.0, -C.Separation);
	return Q.bProximity
			   ? C.Separation <= Q.ProbeDistance
			   : C.Separation < -Q.SpatialTolerance ||
					 (C.Separation <= Q.SpatialTolerance && FVector::DotProduct(Q.End - Q.Start, C.Normal) < -1.e-10);
}
// A 3D frictionless contact QP has an equivalent active set of at most three independent normals.
// Enumerating <= 92 small systems avoids slow PGS convergence in acute wedges; no render-rate/ad-hoc angle fix.
bool ProjectNormalVelocity(const FVector &Base, const FBallContactBatch &Batch, const double *Targets, double Mass,
						   double *Impulses, FVector &Velocity)
{
	const auto TrySet = [&](const int32 *Active, int32 Count, double *OutImpulse, FVector &OutVelocity) {
		double Matrix[3][4] = {};
		for (int32 R = 0; R < Count; ++R)
		{
			for (int32 C = 0; C < Count; ++C)
			{
				Matrix[R][C] = FVector::DotProduct(Batch.Contacts[Active[R]].Normal, Batch.Contacts[Active[C]].Normal);
			}
			const auto &C = Batch.Contacts[Active[R]];
			Matrix[R][Count] = Targets[Active[R]] + FVector::DotProduct(C.SurfaceVelocity - Base, C.Normal);
		}
		for (int32 Col = 0; Col < Count; ++Col)
		{
			int32 Pivot = Col;
			for (int32 R = Col + 1; R < Count; ++R)
			{
				if (FMath::Abs(Matrix[R][Col]) > FMath::Abs(Matrix[Pivot][Col]))
				{
					Pivot = R;
				}
			}
			if (FMath::Abs(Matrix[Pivot][Col]) < 1.e-10)
			{
				return false;
			}
			for (int32 C = Col; C <= Count; ++C)
			{
				Swap(Matrix[Col][C], Matrix[Pivot][C]);
			}
			const double Divisor = Matrix[Col][Col];
			for (int32 C = Col; C <= Count; ++C)
			{
				Matrix[Col][C] /= Divisor;
			}
			for (int32 R = 0; R < Count; ++R)
				if (R != Col)
				{
					const double Scale = Matrix[R][Col];
					for (int32 C = Col; C <= Count; ++C)
					{
						Matrix[R][C] -= Scale * Matrix[Col][C];
					}
				}
		}
		OutVelocity = Base;
		for (int32 I = 0; I < BallLimits::Contacts; ++I)
		{
			OutImpulse[I] = 0;
		}
		for (int32 I = 0; I < Count; ++I)
		{
			if (Matrix[I][Count] < -1.e-7)
			{
				return false;
			}
			const double Lambda = FMath::Max(0.0, Matrix[I][Count]);
			OutImpulse[Active[I]] = Lambda * Mass;
			OutVelocity += Batch.Contacts[Active[I]].Normal * Lambda;
		}
		for (int32 I = 0; I < Batch.Count; ++I)
		{
			if (FVector::DotProduct(OutVelocity - Batch.Contacts[I].SurfaceVelocity, Batch.Contacts[I].Normal) <
				Targets[I] - 1.e-6)
			{
				return false;
			}
		}
		return !OutVelocity.ContainsNaN();
	};
	int32 Active[3] = {};
	if (TrySet(Active, 0, Impulses, Velocity))
	{
		return true;
	}
	for (int32 I = 0; I < Batch.Count; ++I)
	{
		Active[0] = I;
		if (TrySet(Active, 1, Impulses, Velocity))
		{
			return true;
		}
		for (int32 J = I + 1; J < Batch.Count; ++J)
		{
			Active[1] = J;
			if (TrySet(Active, 2, Impulses, Velocity))
			{
				return true;
			}
			for (int32 K = J + 1; K < Batch.Count; ++K)
			{
				Active[2] = K;
				if (TrySet(Active, 3, Impulses, Velocity))
				{
					return true;
				}
			}
		}
	}
	return false;
}
} // namespace

bool FBallCollisionSolver::SphereTOI(const FVector &S, const FVector &D, double Radius, double &OutTime)
{
	const double A = D.SizeSquared(), B = 2 * FVector::DotProduct(S, D), C = S.SizeSquared() - Radius * Radius;
	if (!FMath::IsFinite(A) || !FMath::IsFinite(B) || !FMath::IsFinite(C) || A < 1.e-20)
	{
		return false;
	}
	double Discriminant = B * B - 4 * A * C;
	const double Tolerance = 1.e-13 * FMath::Max(1.0, B * B + FMath::Abs(4 * A * C));
	if (Discriminant < -Tolerance)
	{
		return false;
	}
	Discriminant = FMath::Max(0.0, Discriminant);
	const double Root = FMath::Sqrt(Discriminant);
	const double Q = -0.5 * (B + (B >= 0 ? Root : -Root));
	double T0 = Q / A, T1 = FMath::Abs(Q) > 1.e-30 ? C / Q : T0;
	if (T0 > T1)
	{
		Swap(T0, T1);
	}
	if (T0 < -1.e-10 || T0 > 1 + 1.e-10)
	{
		return false;
	}
	const double Time = FMath::Clamp(T0, 0.0, 1.0);
	// Tangency has no closing impulse; do not produce an endless zero-time grazing contact.
	if (FVector::DotProduct(S + D * Time, D) >= -1.e-10)
	{
		return false;
	}
	OutTime = Time;
	return true;
}

bool FBallCollisionSolver::SweepSphere(const FBallCollisionQuery &Q, const FVector &Center, double Radius,
									   FBallContact &C)
{
	const FVector Offset = Q.Start - Center;
	const double Distance = Offset.Size();
	C.Normal = Distance > 1.e-12 ? Offset / Distance : FVector::UpVector;
	C.Position = Center + C.Normal * Radius;
	C.Separation = Distance - Radius - Q.Radius;
	if (Q.bProximity || C.Separation <= Q.SpatialTolerance)
	{
		return InitialContact(Q, C);
	}
	double Time;
	if (!SphereTOI(Offset, Q.End - Q.Start, Radius + Q.Radius, Time))
	{
		return false;
	}
	C.TimeOfImpact = Time;
	C.Normal = (FMath::Lerp(Q.Start, Q.End, Time) - Center).GetSafeNormal();
	C.Position = Center + C.Normal * Radius;
	C.Separation = C.PenetrationDepth = 0;
	return true;
}

bool FBallCollisionSolver::SweepBox(const FBallCollisionQuery &Q, const FVector &Min, const FVector &Max,
									FBallContact &C)
{
	const FVector Closest = Q.Start.ComponentMax(Min).ComponentMin(Max);
	const FVector Offset = Q.Start - Closest;
	const double Distance = Offset.Size();
	C.Position = Closest;
	C.Separation = Distance - Q.Radius;
	if (Distance > 1.e-12)
	{
		C.Normal = Offset / Distance;
		int32 Outside = 0, Axis = 0, Bits = 0;
		for (int32 I = 0; I < 3; ++I)
		{
			if (Offset[I] != 0)
			{
				++Outside;
				Axis = I;
			}
			if (Q.Start[I] > Max[I])
			{
				Bits |= 1 << I;
			}
		}
		if (Outside == 1)
		{
			C.FeatureId = Axis * 2 + ((Bits >> Axis) & 1);
		}
		else if (Outside == 3)
		{
			C.FeatureId = 18 + Bits;
		}
		else
		{
			for (int32 I = 0; I < 3; ++I)
			{
				if (Offset[I] == 0)
				{
					Axis = I;
				}
			}
			C.FeatureId = 6 + Axis * 4 + ((Bits >> ((Axis + 1) % 3)) & 1) * 2 + ((Bits >> ((Axis + 2) % 3)) & 1);
		}
	}
	else
	{
		double Best = TNumericLimits<double>::Max();
		for (int32 Axis = 0; Axis < 3; ++Axis)
		{
			for (int32 Side = 0; Side < 2; ++Side)
			{
				const double Gap = Side ? Max[Axis] - Q.Start[Axis] : Q.Start[Axis] - Min[Axis];
				if (Gap < Best)
				{
					Best = Gap;
					C.Normal = FVector::ZeroVector;
					C.Normal[Axis] = Side ? 1 : -1;
					C.FeatureId = Axis * 2 + Side;
				}
			}
		}
		C.Separation = -Best - Q.Radius;
		C.Position = Q.Start + C.Normal * Best;
	}
	if (Q.bProximity || C.Separation <= Q.SpatialTolerance)
	{
		return InitialContact(Q, C);
	}
	const FVector Delta = Q.End - Q.Start;
	bool bFound = false;
	double BestTime = 2;
	FBallContact Best = C;
	const auto Candidate = [&](double Time, const FVector &Point, uint32 Feature) {
		if (Time < 0 || Time > 1 || Time > BestTime)
		{
			return;
		}
		const FVector Normal = (Q.Start + Delta * Time - Point).GetSafeNormal();
		if (Normal.IsNearlyZero() || FVector::DotProduct(Delta, Normal) >= -1.e-10)
		{
			return;
		}
		if (Time == BestTime && Feature >= Best.FeatureId)
		{
			return;
		}
		bFound = true;
		BestTime = Time;
		Best.TimeOfImpact = Time;
		Best.Position = Point;
		Best.Normal = Normal;
		Best.FeatureId = Feature;
		Best.Separation = Best.PenetrationDepth = 0;
	};
	// Flat faces; an expanded AABB alone would give incorrect normals/TOIs at rounded edges.
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		if (FMath::Abs(Delta[Axis]) < 1.e-15)
		{
			continue;
		}
		for (int32 Side = 0; Side < 2; ++Side)
		{
			const double Face = Side ? Max[Axis] : Min[Axis];
			const double Time = (Face + (Side ? Q.Radius : -Q.Radius) - Q.Start[Axis]) / Delta[Axis];
			const FVector Center = Q.Start + Delta * Time;
			const int32 A = (Axis + 1) % 3, B = (Axis + 2) % 3;
			if (Center[A] < Min[A] || Center[A] > Max[A] || Center[B] < Min[B] || Center[B] > Max[B])
			{
				continue;
			}
			FVector Point = Center;
			Point[Axis] = Face;
			Candidate(Time, Point, Axis * 2 + Side);
		}
	}
	// Twelve finite edge cylinders, with outer-region tests.
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		const int32 A = (Axis + 1) % 3, B = (Axis + 2) % 3;
		for (int32 SA = 0; SA < 2; ++SA)
			for (int32 SB = 0; SB < 2; ++SB)
			{
				FVector Point = Min;
				Point[A] = SA ? Max[A] : Min[A];
				Point[B] = SB ? Max[B] : Min[B];
				FVector Relative = Q.Start - Point, Motion = Delta;
				Relative[Axis] = Motion[Axis] = 0;
				double Time;
				if (!SphereTOI(Relative, Motion, Q.Radius, Time))
				{
					continue;
				}
				const FVector Center = Q.Start + Delta * Time;
				if (Center[Axis] < Min[Axis] || Center[Axis] > Max[Axis] ||
					(SA ? Center[A] < Max[A] : Center[A] > Min[A]) || (SB ? Center[B] < Max[B] : Center[B] > Min[B]))
				{
					continue;
				}
				Point[Axis] = Center[Axis];
				Candidate(Time, Point, 6 + Axis * 4 + SA * 2 + SB);
			}
	}
	for (int32 Bits = 0; Bits < 8; ++Bits)
	{
		FVector Point;
		for (int32 A = 0; A < 3; ++A)
		{
			Point[A] = Bits & (1 << A) ? Max[A] : Min[A];
		}
		double Time;
		if (!SphereTOI(Q.Start - Point, Delta, Q.Radius, Time))
		{
			continue;
		}
		const FVector Center = Q.Start + Delta * Time;
		bool bOutside = true;
		for (int32 A = 0; A < 3; ++A)
		{
			bOutside &= Bits & (1 << A) ? Center[A] >= Max[A] : Center[A] <= Min[A];
		}
		if (bOutside)
		{
			Candidate(Time, Point, 18 + Bits);
		}
	}
	if (bFound)
	{
		C = Best;
	}
	return bFound;
}

bool FBallCollisionSolver::RefreshContacts(FBallState &State, const FBallContactBatch &Batch, bool bSupporting)
{
	for (int32 I = 0; I < State.ContactCount; ++I)
	{
		State.Contacts[I].bTouching = State.Contacts[I].bSupporting = false;
	}
	for (int32 I = 0; I < Batch.Count; ++I)
	{
		bool bNew;
		const FBallContact &C = Batch.Contacts[I];
		const int32 Index = Cache(State, C, bNew);
		if (Index == INDEX_NONE)
		{
			return false;
		}
		auto &Cached = State.Contacts[Index];
		Cached.Normal = C.Normal;
		Cached.ContactPoint = C.Position;
		Cached.Separation = C.Separation;
		Cached.bTouching = true;
		Cached.bSupporting = bSupporting;
	}
	return true;
}

bool FBallCollisionSolver::HasFeasibleVelocity(const FBallContactBatch &Batch)
{
	double Targets[BallLimits::Contacts] = {}, Impulses[BallLimits::Contacts] = {};
	FVector Velocity;
	return ProjectNormalVelocity(FVector::ZeroVector, Batch, Targets, 1, Impulses, Velocity);
}

bool FBallCollisionSolver::Separate(FBallState &State, const FBallContactBatch &Batch, double Skin)
{
	FBallContactBatch Static = Batch;
	double Targets[BallLimits::Contacts] = {}, Impulses[BallLimits::Contacts] = {};
	for (int32 I = 0; I < Static.Count; ++I)
	{
		Static.Contacts[I].SurfaceVelocity = FVector::ZeroVector;
		Targets[I] = Static.Contacts[I].PenetrationDepth + Skin - FMath::Max(0.0, Static.Contacts[I].Separation);
	}
	FVector Translation;
	if (!ProjectNormalVelocity(FVector::ZeroVector, Static, Targets, 1, Impulses, Translation))
	{
		return false;
	}
	State.Position += Translation;
	return !State.Position.ContainsNaN();
}

bool FBallCollisionSolver::CanSupport(const FBallContactBatch &Batch, const FVector &Acceleration,
									  const FBallPhysicsConfig &Config, FVector &OutNormal,
									  FBallColliderId &OutCollider)
{
	if (Batch.Count == 0)
	{
		return false;
	}
	FVector Remaining;
	double Targets[BallLimits::Contacts] = {}, Impulses[BallLimits::Contacts] = {};
	if (!ProjectNormalVelocity(Acceleration, Batch, Targets, 1, Impulses, Remaining))
	{
		return false;
	}
	double NormalAcceleration = 0, FrictionCapacity = 0;
	FVector NormalSum = FVector::ZeroVector;
	int32 Strongest = 0;
	for (int32 I = 0; I < Batch.Count; ++I)
	{
		const auto &C = Batch.Contacts[I];
		if (!C.SurfaceVelocity.IsNearlyZero(1.e-6))
		{
			return false;
		}
		NormalAcceleration += Impulses[I];
		NormalSum += C.Normal * Impulses[I];
		FrictionCapacity += Friction(C, Config) * Impulses[I];
		if (Impulses[I] > Impulses[Strongest])
		{
			Strongest = I;
		}
	}
	OutNormal = NormalSum.GetSafeNormal();
	if (OutNormal.IsNearlyZero())
	{
		OutNormal = Batch.Contacts[0].Normal;
	}
	OutCollider = Batch.Contacts[Strongest].Collider;
	// Friction alone cannot hold a free sphere on a slope without producing rolling torque.
	return Remaining.Size() <= FMath::Min(FrictionCapacity, Config.RollingResistance * NormalAcceleration) + 1.e-6;
}

EBallStepResult FBallCollisionSolver::Resolve(FBallState &State, const FBallContactBatch &Batch,
											  const FBallPhysicsConfig &Config, double Fraction,
											  FBallStepOutput &Output, bool bRecordFacts)
{
	if (Batch.Count < 1 || Batch.Count > BallLimits::Contacts)
	{
		return EBallStepResult::QueryFailure;
	}
	const FVector Incoming = State.LinearVelocity, IncomingAngular = State.AngularVelocity;
	const double InitialEnergy = Energy(State, Config), InverseMass = 1.0 / Config.Mass;
	const double InverseInertia = 1.0 / Config.GetSolidSphereInertia();
	double Jn[BallLimits::Contacts] = {}, Target[BallLimits::Contacts] = {};
	FVector Jt[BallLimits::Contacts];
	bool bPassive = true;
	for (int32 I = 0; I < Batch.Count; ++I)
	{
		bPassive &= Batch.Contacts[I].SurfaceVelocity.IsNearlyZero(1.e-9);
	}
	for (int32 Attempt = 0; Attempt < 2; ++Attempt)
	{
		State.LinearVelocity = Incoming;
		State.AngularVelocity = IncomingAngular;
		for (int32 I = 0; I < Batch.Count; ++I)
		{
			Jn[I] = 0;
			Jt[I] = FVector::ZeroVector;
			const auto &C = Batch.Contacts[I];
			const double Vn = FVector::DotProduct(Incoming - C.SurfaceVelocity, C.Normal);
			Target[I] = Attempt == 0 && C.bAllowImpact && C.PenetrationDepth <= Config.PenetrationTolerance &&
								Vn < -Config.MinimumImpactSpeed
							? -Restitution(C, Config) * Vn
							: 0;
		}
		const auto CorrectNormals = [&]() {
			FVector Base = State.LinearVelocity;
			for (int32 I = 0; I < Batch.Count; ++I)
			{
				Base -= Batch.Contacts[I].Normal * (Jn[I] * InverseMass);
			}
			return ProjectNormalVelocity(Base, Batch, Target, Config.Mass, Jn, State.LinearVelocity);
		};
		bool bFeasible = true;
		for (int32 Pass = 0; Pass < Config.ManifoldIterations; ++Pass)
		{
			if (!CorrectNormals())
			{
				bFeasible = false;
				break;
			}
			for (int32 I = 0; I < Batch.Count; ++I)
			{
				const auto &C = Batch.Contacts[I];
				const FVector Arm = -C.Normal * Config.Radius;
				const FVector Relative =
					State.LinearVelocity + FVector::CrossProduct(State.AngularVelocity, Arm) - C.SurfaceVelocity;
				const FVector Tangent = Relative - C.Normal * FVector::DotProduct(Relative, C.Normal);
				const FVector NewJt = (Jt[I] - Tangent / (InverseMass + Config.Radius * Config.Radius * InverseInertia))
										  .GetClampedToMaxSize(Friction(C, Config) * Jn[I]);
				const FVector DeltaJ = NewJt - Jt[I];
				Jt[I] = NewJt;
				State.LinearVelocity += DeltaJ * InverseMass;
				State.AngularVelocity += FVector::CrossProduct(Arm, DeltaJ) * InverseInertia;
			}
		}
		// Friction at one contact affects another contact's normal. End with bounded cone/non-penetration feasibility.
		if (bFeasible)
		{
			for (int32 Pass = 0; Pass < Config.ManifoldIterations; ++Pass)
			{
				if (!CorrectNormals())
				{
					bFeasible = false;
					break;
				}
				bool bClamped = false;
				for (int32 I = 0; I < Batch.Count; ++I)
				{
					const auto &C = Batch.Contacts[I];
					const double Limit = Friction(C, Config) * Jn[I];
					if (Jt[I].Size() <= Limit + 1.e-8)
					{
						continue;
					}
					const FVector NewJt = Jt[I].GetClampedToMaxSize(Limit), Delta = NewJt - Jt[I];
					Jt[I] = NewJt;
					State.LinearVelocity += Delta * InverseMass;
					State.AngularVelocity += FVector::CrossProduct(-C.Normal * Config.Radius, Delta) * InverseInertia;
					bClamped = true;
				}
				if (!bClamped)
				{
					break;
				}
				if (Pass + 1 == Config.ManifoldIterations)
				{
					bFeasible = false;
				}
			}
		}
		if (!bFeasible)
		{
			if (Attempt == 0)
			{
				++Output.Diagnostics.RestitutionFallbacks;
				continue;
			}
			return EBallStepResult::CollisionLimit;
		}
		if (!bPassive || Energy(State, Config) <= InitialEnergy + FMath::Max(1.e-8, InitialEnergy * 1.e-10))
		{
			break;
		}
		// Coupled restitution targets can be mutually infeasible in acute corners. Retry the SAME manifold
		// inelastically rather than invent energy or independently reflect each plane repeatedly.
		if (Attempt == 1)
		{
			return EBallStepResult::NumericFailure;
		}
		++Output.Diagnostics.RestitutionFallbacks;
	}
	for (int32 I = 0; I < Batch.Count; ++I)
	{
		const auto &C = Batch.Contacts[I];
		if (FVector::DotProduct(State.LinearVelocity - C.SurfaceVelocity, C.Normal) < -1.e-4)
		{
			return EBallStepResult::CollisionLimit;
		}
		bool bNew;
		const int32 Index = Cache(State, C, bNew);
		if (Index == INDEX_NONE || (bRecordFacts && Output.ContactCount == BallLimits::StepEvents))
		{
			return EBallStepResult::ContactCapacity;
		}
		auto &Cached = State.Contacts[Index];
		const double ClosingSpeed = -FVector::DotProduct(Incoming - C.SurfaceVelocity, C.Normal);
		Cached.Normal = C.Normal;
		Cached.ContactPoint = C.Position;
		Cached.Separation = C.Separation;
		Cached.bTouching = true;
		Cached.AccumulatedNormalImpulse = Jn[I];
		Cached.AccumulatedTangentImpulse = Jt[I];
		Cached.bRestitutionApplied = Target[I] > 0;
		FBallContact Reported = C;
		Reported.TimeOfImpact = Fraction;
		if (bRecordFacts)
		{
			Output.Contacts[Output.ContactCount++] = Reported;
		}
		if (bRecordFacts && C.bAllowImpact && ClosingSpeed > Config.MinimumImpactSpeed && Jn[I] > 0 &&
			C.PenetrationDepth <= Config.PenetrationTolerance)
		{
			if (Output.EventCount == BallLimits::StepEvents || State.NextEventOrdinal == MAX_uint32 ||
				Cached.Episode == MAX_uint32)
			{
				return EBallStepResult::ContactCapacity;
			}
			if (!bNew)
			{
				++Cached.Episode;
			}
			Cached.LastImpactFrame = State.SimulationFrame;
			auto &Event = Output.Events[Output.EventCount++];
			Event.Ball = State.Identity;
			Event.Category = C.Category;
			Event.Time = {State.SimulationFrame, FMath::Min(Fraction, 1.0 - 1.e-12)};
			Event.Collider = C.Collider;
			Event.FeatureId = C.FeatureId;
			Event.Episode = Cached.Episode;
			Event.Ordinal = State.NextEventOrdinal++;
			Event.Position = C.Position;
			Event.Normal = C.Normal;
			Event.IncomingVelocity = Incoming;
			Event.OutgoingVelocity = State.LinearVelocity;
			Event.IncomingAngularVelocity = IncomingAngular;
			Event.OutgoingAngularVelocity = State.AngularVelocity;
			Event.NormalImpulse = Jn[I];
			Event.Participant = C.Participant;
			Event.OriginRevision = C.OriginRevision;
			Event.HandSide = C.HandSide;
			Event.FirstSample = C.FirstSample;
			Event.SecondSample = C.SecondSample;
		}
	}
	return EBallStepResult::Success;
}
