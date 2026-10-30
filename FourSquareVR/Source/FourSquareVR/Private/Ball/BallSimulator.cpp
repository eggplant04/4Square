#include "Ball/BallSimulator.h"
#include "Ball/BallIntegrator.h"
#include "Ball/BallCollisionSolver.h"
#include "Ball/BallHandCollisionSolver.h"
#include "Ball/BallMotionContactPolicy.h"
#include "HAL/PlatformTime.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

EBallStepResult FBallSimulator::SimulateStep(FBallState &State, const FBallStepInputs &Inputs,
											 const FBallPhysicsConfig &Config, const IBallCollisionWorld &World,
											 FBallStepOutput &Output)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Handball_SimulateStep);
	const double Start = FPlatformTime::Seconds();
	Output = {};
	const auto Finish = [&](EBallStepResult Result) {
		Output.Result = Result;
		if (Result != EBallStepResult::Success)
		{
			Output.EventCount = Output.ContactCount = 0;
		}
		Output.Diagnostics.DurationMicroseconds = (FPlatformTime::Seconds() - Start) * 1.e6;
		Counters.RecordStep(Output.Diagnostics.DurationMicroseconds, Result == EBallStepResult::Success,
							Output.Diagnostics.Queries, Output.Diagnostics.Microintervals);
		return Result;
	};
	if (!Config.IsValid())
	{
		return Finish(EBallStepResult::InvalidConfiguration);
	}
	if (!State.IsValid() || State.SimulationFrame >= MAX_uint64 - 1)
	{
		return Finish(EBallStepResult::InvalidState);
	}
	if (!Inputs.IsValid() || Inputs.SimulationFrame != State.SimulationFrame ||
		Inputs.ConfigVersion != State.ConfigVersion || State.ConfigVersion != static_cast<uint32>(Config.Version) ||
		Inputs.MaterialVersion != State.MaterialVersion)
	{
		return Finish(EBallStepResult::InvalidInputs);
	}
	if (World.GetRevision() != State.EnvironmentRevision || Inputs.EnvironmentRevision != State.EnvironmentRevision ||
		!World.ValidateInputs(Inputs))
	{
		return Finish(EBallStepResult::IncompatibleEnvironment);
	}
	FVector Acceleration = Config.Gravity, AngularAcceleration = FVector::ZeroVector;
	for (int32 I = 0; I < Inputs.ForceCount; ++I)
	{
		Acceleration += Inputs.Forces[I].LinearAcceleration;
		AngularAcceleration += Inputs.Forces[I].AngularAcceleration;
	}
	const double Dt = Config.GetFixedDeltaTime();
	const FBallMotionFrame *Motion = World.GetMotionFrame(Inputs);
	if (Motion && (Motion->Epoch != State.Identity.Epoch || FMath::Abs(Motion->DeltaTime - Dt) > 1.e-12))
	{
		return Finish(EBallStepResult::IncompatibleEnvironment);
	}
	const double Required = FMath::Sqrt(Acceleration.Size() * Dt * Dt / (8 * Config.CCDTolerance));
	if (!FMath::IsFinite(Required) || AngularAcceleration.ContainsNaN())
	{
		return Finish(EBallStepResult::NumericFailure);
	}
	if (Required > Config.MaximumMicrointervals)
	{
		return Finish(EBallStepResult::SubdivisionLimit);
	}
	const int32 InitialIntervals = FMath::Max(1, FMath::CeilToInt32(Required));
	const auto MakeQuery = [&](FBallState &S, double BeginFraction, double EndFraction) {
		FBallCollisionQuery Q;
		Q.Start = Q.End = S.Position;
		Q.Radius = Config.Radius;
		Q.Time = {S.SimulationFrame, BeginFraction};
		Q.EndFraction = EndFraction;
		Q.SpatialTolerance = Config.CCDTolerance;
		Q.Channel = Config.QueryChannel;
		Q.RequiredProfile = Config.RequiredCollisionProfile;
		Q.bTraceComplex = Config.bTraceComplex;
		Q.bUsePhysicalMaterials = Config.bUsePhysicalMaterials;
		Q.MaximumCandidates = Config.MaximumWorldCandidates;
		Q.StepFrame = Inputs.SimulationFrame;
		Q.MotionTimebase = Inputs.MotionTimebase;
		Q.MotionRevision = Inputs.MotionRevision;
		if (Motion)
		{
			Q.IgnoredMotionMask = FBallMotionContactPolicy::PrepareQuery(
				S, *Motion, BeginFraction, Config.Radius,
				FMath::Max(Config.DynamicContactReleaseDistance, 2 * Config.PenetrationTolerance));
		}
		return Q;
	};
	const auto Gather = [&](const FBallCollisionQuery &Q, FBallContactBatch &Batch) {
		++Output.Diagnostics.Queries;
		const auto Result = World.GatherContacts(Q, Batch);
		if (Result == EBallQueryResult::CapacityExceeded)
		{
			return EBallStepResult::ContactCapacity;
		}
		if (Result != EBallQueryResult::Clear && Result != EBallQueryResult::Contact)
		{
			return EBallStepResult::QueryFailure;
		}
		if (Batch.Count < 0 || Batch.Count > BallLimits::Contacts ||
			(Result == EBallQueryResult::Contact) != (Batch.Count > 0))
		{
			return EBallStepResult::QueryFailure;
		}
		FBallContactBatch Validated;
		for (int32 I = 0; I < Batch.Count; ++I)
		{
			if (!Validated.Add(Batch.Contacts[I], Q))
			{
				return EBallStepResult::QueryFailure;
			}
		}
		Validated.Sort();
		Batch = Validated;
		return EBallStepResult::Success;
	};
	const auto Probe = [&](FBallState &S, FBallContactBatch &Batch, double Fraction) {
		auto Q = MakeQuery(S, FMath::Min(Fraction, 1.0 - 1.e-12), 1);
		Q.bProximity = true;
		Q.ProbeDistance = 2 * FMath::Max(Config.PenetrationTolerance, Config.CCDTolerance);
		if (Fraction >= 1)
		{
			Q.Time.Frame = S.SimulationFrame + 1;
			Q.Time.Fraction = 0;
		}
		else
		{
			Q.Time.Fraction = Fraction;
		}
		if (Motion)
		{
			Q.IgnoredMotionMask = FBallMotionContactPolicy::PrepareQuery(
				S, *Motion, Fraction, Config.Radius,
				FMath::Max(Config.DynamicContactReleaseDistance, 2 * Config.PenetrationTolerance));
		}
		return Gather(Q, Batch);
	};
	// Retry only bounded geometric/iteration failures with smaller deterministic microintervals.
	// Fixed dt never changes; no elapsed time or collision remainder is silently lost.
	for (int32 Intervals = InitialIntervals; Intervals <= Config.MaximumMicrointervals; Intervals *= 2)
	{
		FBallState Candidate = State;
		if (Motion)
		{
			FBallMotionContactPolicy::RetireAbsent(Candidate, *Motion);
		}
		Output.ContactCount = Output.EventCount = 0;
		EBallStepResult Result = EBallStepResult::Success;
		FBallContactBatch Near;
		Result = Probe(Candidate, Near, 0);
		if (Result != EBallStepResult::Success)
		{
			return Finish(Result);
		}
		FVector SupportNormal;
		FBallColliderId SupportCollider;
		bool bBalanced = FBallCollisionSolver::CanSupport(Near, Acceleration, Config, SupportNormal, SupportCollider);
		bool bDeep = false;
		for (int32 I = 0; I < Near.Count; ++I)
		{
			bDeep |= Near.Contacts[I].PenetrationDepth > Config.PenetrationTolerance;
		}
		if (Candidate.Activity == EBallActivityState::Resting && bBalanced && !bDeep &&
			AngularAcceleration.IsNearlyZero() && Candidate.LinearVelocity.Size() <= Config.WakeSpeed &&
			Candidate.AngularVelocity.Size() <= Config.SleepAngularSpeed &&
			(!Motion || !FBallHandCollisionSolver::CouldReach(
							*Motion, Candidate.Position,
							Config.Radius + FMath::Max(Config.CCDTolerance, Config.PenetrationTolerance))))
		{
			if (!FBallCollisionSolver::RefreshContacts(Candidate, Near, true))
			{
				return Finish(EBallStepResult::ContactCapacity);
			}
			Candidate.LinearVelocity = Candidate.AngularVelocity = FVector::ZeroVector;
			Candidate.SupportNormal = SupportNormal;
			Candidate.SupportCollider = SupportCollider;
			Candidate.bHasSupport = true;
			Candidate.SupportVelocity = FVector::ZeroVector;
			++Candidate.SimulationFrame;
			State = Candidate;
			return Finish(EBallStepResult::Success);
		}
		Candidate.Activity = EBallActivityState::Active;
		if (!bBalanced)
		{
			Candidate.SleepDwellTime = 0;
		}
		const double H = Dt / Intervals;
		uint32 IterationsThisAttempt = 0;
		for (int32 Micro = 0; Micro < Intervals && Result == EBallStepResult::Success; ++Micro)
		{
			++Output.Diagnostics.Microintervals;
			FVector ForceBaseLinear = Candidate.LinearVelocity, ForceBaseAngular = Candidate.AngularVelocity;
			double ForceStartElapsed = 0, ForceDuration = H;
			if (!FBallIntegrator::Kick(Candidate, Acceleration, AngularAcceleration, Config, H))
			{
				Result = EBallStepResult::NumericFailure;
				break;
			}
			double Remaining = H, Elapsed = 0;
			while (Remaining > 1.e-12 && Result == EBallStepResult::Success)
			{
				if (++Output.Diagnostics.MotionSlices > 256u * uint32(Config.MaximumMicrointervals))
				{
					Result = EBallStepResult::SubdivisionLimit;
					break;
				}
				const double Fraction = (Micro * H + Elapsed) / Dt;
				auto Q = MakeQuery(Candidate, FMath::Min(Fraction, 1.0 - 1.e-12),
								   static_cast<double>(Micro + 1) / Intervals);
				if (Motion)
				{
					Q.EndFraction = FBallHandCollisionSolver::NextBoundary(*Motion, Fraction, Q.EndFraction);
				}
				const double Slice = FMath::Min(Remaining, (Q.EndFraction - Fraction) * Dt);
				if (Slice <= 1.e-14)
				{
					Result = EBallStepResult::SubdivisionLimit;
					break;
				}
				Q.End = Q.Start + Candidate.LinearVelocity * Slice;
				const double MaximumTravel =
					Motion ? FBallHandCollisionSolver::MaximumRelativeTravel(*Motion, Q) : (Q.End - Q.Start).Size();
				Q.TOIFractionTolerance = FMath::Min3(1.e-3, Config.TOITolerance / Slice,
													 Config.CCDTolerance / FMath::Max(1.e-12, MaximumTravel));
				FBallContactBatch Batch;
				Result = Gather(Q, Batch);
				if (Result != EBallStepResult::Success)
				{
					break;
				}
				if (Batch.Count == 0)
				{
					if (!FBallIntegrator::Drift(Candidate, Slice))
					{
						Result = EBallStepResult::NumericFailure;
					}
					Elapsed += Slice;
					Remaining -= Slice;
					continue;
				}
				++Output.Diagnostics.CollisionIterations;
				if (++IterationsThisAttempt > static_cast<uint32>(Config.MaximumCollisionIterations))
				{
					Result = EBallStepResult::CollisionLimit;
					break;
				}
				double TOI = Batch.Contacts[0].TimeOfImpact;
				for (int32 I = 1; I < Batch.Count; ++I)
				{
					TOI = FMath::Min(TOI, Batch.Contacts[I].TimeOfImpact);
				}
				const double Travel = Slice * TOI;
				if (!FBallIntegrator::Drift(Candidate, Travel))
				{
					Result = EBallStepResult::NumericFailure;
					break;
				}
				Elapsed += Travel;
				Remaining -= Travel;
				const FVector PredictedLinear = Candidate.LinearVelocity, PredictedAngular = Candidate.AngularVelocity;
				// Use velocity at impact, not the future endpoint of the pending force stage.
				const double ForceFraction =
					ForceDuration > 1.e-12 ? FMath::Clamp((Elapsed - ForceStartElapsed) / ForceDuration, 0.0, 1.0) : 1;
				Candidate.LinearVelocity = FMath::Lerp(ForceBaseLinear, Candidate.LinearVelocity, ForceFraction);
				Candidate.AngularVelocity = FMath::Lerp(ForceBaseAngular, Candidate.AngularVelocity, ForceFraction);
				// A tangential/resting face is still a constraint after a different face's impulse.
				FBallContactBatch Neighbours;
				Result = Probe(Candidate, Neighbours, (Micro * H + Elapsed) / Dt);
				if (Result != EBallStepResult::Success)
				{
					break;
				}
				FBallCollisionQuery MergeQuery;
				MergeQuery.bProximity = true;
				for (int32 I = 0; I < Neighbours.Count; ++I)
				{
					const auto &C = Neighbours.Contacts[I];
					if (C.Separation > Config.PenetrationTolerance)
					{
						continue;
					}
					bool bDuplicate = false;
					for (int32 J = 0; J < Batch.Count; ++J)
					{
						bDuplicate |= Batch.Contacts[J].Collider == C.Collider &&
									  FVector::DotProduct(Batch.Contacts[J].Normal, C.Normal) > 1 - 1.e-10;
					}
					if (!bDuplicate && !Batch.Add(C, MergeQuery))
					{
						Result = EBallStepResult::ContactCapacity;
						break;
					}
				}
				if (Result != EBallStepResult::Success)
				{
					break;
				}
				Batch.Sort();
				if (Motion)
				{
					FBallMotionContactPolicy::FilterConflicts(Candidate, Batch, Config.PenetrationTolerance, Output);
				}
				if (!Batch.Count)
				{
					Candidate.LinearVelocity = PredictedLinear;
					Candidate.AngularVelocity = PredictedAngular;
					continue; // Requery with blocked sources excluded, preserving later static hits.
				}
				const int32 FirstEvent = Output.EventCount;
				Result = FBallCollisionSolver::Resolve(Candidate, Batch, Config, (Micro * H + Elapsed) / Dt, Output);
				if (Result != EBallStepResult::Success)
				{
					break;
				}
				if (Motion)
				{
					FBallMotionContactPolicy::CompleteResponse(Candidate, Batch, FirstEvent, Output);
				}
				for (int32 I = 0; I < Batch.Count; ++I)
				{
					const auto &C = Batch.Contacts[I];
					if (C.PenetrationDepth > 0)
					{
						++Output.Diagnostics.PenetrationCorrections;
					}
				}
				if (!FBallCollisionSolver::Separate(Candidate, Batch, Config.PenetrationTolerance))
				{
					Result = EBallStepResult::PenetrationFailure;
				}
				if (Result != EBallStepResult::Success)
				{
					break;
				}
				ForceBaseLinear = Candidate.LinearVelocity;
				ForceBaseAngular = Candidate.AngularVelocity;
				ForceStartElapsed = Elapsed;
				ForceDuration = Remaining;
				if (Remaining > 1.e-12)
				{
					if (!FBallIntegrator::Kick(Candidate, Acceleration, AngularAcceleration, Config, Remaining))
					{
						Result = EBallStepResult::NumericFailure;
						break;
					}
					bool bReentry = false;
					FBallContactBatch Support = Batch;
					for (int32 I = 0; I < Support.Count; ++I)
					{
						auto &C = Support.Contacts[I];
						C.bAllowImpact = false;
						bReentry |=
							FVector::DotProduct(Candidate.LinearVelocity - C.SurfaceVelocity, C.Normal) < -1.e-6;
					}
					if (bReentry)
					{
						++Output.Diagnostics.SupportForceSolves;
						Result = FBallCollisionSolver::Resolve(Candidate, Support, Config, (Micro * H + Elapsed) / Dt,
															   Output, false);
					}
				}
			}
		}
		if (Result == EBallStepResult::CollisionLimit || Result == EBallStepResult::PenetrationFailure)
		{
			if (Intervals * 2 <= Config.MaximumMicrointervals)
			{
				continue;
			}
			return Finish(Result);
		}
		if (Result != EBallStepResult::Success)
		{
			return Finish(Result);
		}
		Result = Probe(Candidate, Near, 1);
		if (Result != EBallStepResult::Success)
		{
			return Finish(Result);
		}
		bool bStillPenetrating = false;
		for (int32 I = 0; I < Near.Count; ++I)
		{
			bStillPenetrating |= Near.Contacts[I].PenetrationDepth > Config.PenetrationTolerance + 1.e-6;
		}
		if (bStillPenetrating)
		{
			if (Intervals * 2 <= Config.MaximumMicrointervals)
			{
				continue;
			}
			return Finish(EBallStepResult::PenetrationFailure);
		}
		bBalanced = FBallCollisionSolver::CanSupport(Near, Acceleration, Config, SupportNormal, SupportCollider);
		if (!FBallCollisionSolver::RefreshContacts(Candidate, Near, bBalanced))
		{
			return Finish(EBallStepResult::ContactCapacity);
		}
		Candidate.bHasSupport = bBalanced;
		Candidate.SupportCollider = bBalanced ? SupportCollider : FBallColliderId{};
		Candidate.SupportNormal = bBalanced ? SupportNormal : FVector::UpVector;
		Candidate.SupportVelocity = FVector::ZeroVector;
		if (bBalanced)
		{
			// Rolling/spin resistance dissipates energy; it never synthesizes visual rotation.
			const double Deceleration = Config.RollingResistance * Acceleration.Size() * Dt;
			const double Speed = Candidate.LinearVelocity.Size();
			if (Speed > 0)
			{
				Candidate.LinearVelocity *= FMath::Max(0.0, 1 - Deceleration / Speed);
			}
			const double Spin = Candidate.AngularVelocity.Size();
			if (Spin > 0)
			{
				Candidate.AngularVelocity *=
					FMath::Max(0.0, 1 - Deceleration / (Config.Radius * Spin)) * FMath::Exp(-Config.SpinFriction * Dt);
			}
		}
		if (bBalanced && AngularAcceleration.IsNearlyZero() && Candidate.LinearVelocity.Size() <= Config.SleepSpeed &&
			Candidate.AngularVelocity.Size() <= Config.SleepAngularSpeed)
		{
			Candidate.SleepDwellTime += Dt;
			if (Candidate.SleepDwellTime + 1.e-12 >= Config.SleepDwellSeconds)
			{
				Candidate.Activity = EBallActivityState::Resting;
				Candidate.LinearVelocity = Candidate.AngularVelocity = FVector::ZeroVector;
			}
		}
		else
		{
			Candidate.SleepDwellTime = 0;
		}
		++Candidate.SimulationFrame;
		if (!Candidate.IsValid())
		{
			return Finish(EBallStepResult::NumericFailure);
		}
		State = Candidate;
		return Finish(EBallStepResult::Success);
	}
	return Finish(EBallStepResult::CollisionLimit);
}
