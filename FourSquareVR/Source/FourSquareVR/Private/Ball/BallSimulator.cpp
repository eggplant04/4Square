#include "Ball/BallSimulator.h"
#include "Ball/BallIntegrator.h"
#include "HAL/PlatformTime.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

EBallStepResult FBallSimulator::SimulateStep(FBallState& State, const FBallStepInputs& Inputs,
	const FBallPhysicsConfig& Config, const IBallCollisionWorld& World, FBallStepOutput& Output)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Handball_SimulateStep);
	const double Start = FPlatformTime::Seconds();
	Output = {};
	const auto Finish = [&](EBallStepResult Result)
	{
		Output.Result = Result;
		Output.Diagnostics.DurationMicroseconds = (FPlatformTime::Seconds() - Start) * 1.e6;
		Counters.RecordStep(Output.Diagnostics.DurationMicroseconds, Result == EBallStepResult::Success,
			Output.Diagnostics.Queries, Output.Diagnostics.Microintervals);
		return Result;
	};
	if (!Config.IsValid()) { return Finish(EBallStepResult::InvalidConfiguration); }
	if (!State.IsValid() || State.SimulationFrame >= MAX_uint64 - 1) { return Finish(EBallStepResult::InvalidState); }
	if (!Inputs.IsValid() || Inputs.SimulationFrame != State.SimulationFrame
		|| Inputs.ConfigVersion != State.ConfigVersion || State.ConfigVersion != static_cast<uint32>(Config.Version)
		|| Inputs.MaterialVersion != State.MaterialVersion) { return Finish(EBallStepResult::InvalidInputs); }
	if (World.GetRevision() != State.EnvironmentRevision || Inputs.EnvironmentRevision != State.EnvironmentRevision)
	{ return Finish(EBallStepResult::IncompatibleEnvironment); }
	if (State.Activity != EBallActivityState::Active || State.ContactCount != 0 || State.bHasSupport)
	{ return Finish(EBallStepResult::ContactSolverRequired); }

	FVector LinearAcceleration = Config.Gravity;
	FVector AngularAcceleration = FVector::ZeroVector;
	for (int32 I = 0; I < Inputs.ForceCount; ++I)
	{
		LinearAcceleration += Inputs.Forces[I].LinearAcceleration;
		AngularAcceleration += Inputs.Forces[I].AngularAcceleration;
	}
	const double DeltaTime = Config.GetFixedDeltaTime();
	const double Required = FMath::Sqrt(LinearAcceleration.Size() * DeltaTime * DeltaTime / (8 * Config.CCDTolerance));
	if (!FMath::IsFinite(Required)) { return Finish(EBallStepResult::NumericFailure); }
	if (Required > Config.MaximumMicrointervals) { return Finish(EBallStepResult::SubdivisionLimit); }
	const int32 Count = FMath::Max(1, FMath::CeilToInt32(Required));
	const double Interval = DeltaTime / Count;
	FBallState Candidate = State; // Transactional: any query/math failure leaves all live persistent state untouched.
	for (int32 I = 0; I < Count; ++I)
	{
		++Output.Diagnostics.Microintervals;
		if (!FBallIntegrator::Kick(Candidate, LinearAcceleration, AngularAcceleration, Config, Interval))
		{ return Finish(EBallStepResult::NumericFailure); }
		FBallCollisionQuery Query;
		Query.Start = Candidate.Position;
		Query.End = Candidate.Position + Candidate.LinearVelocity * Interval;
		Query.Radius = Config.Radius;
		Query.Time = {State.SimulationFrame, static_cast<double>(I) / Count};
		Query.EndFraction = static_cast<double>(I + 1) / Count;
		FBallContact Contact;
		++Output.Diagnostics.Queries;
		const EBallQueryResult QueryResult = World.SweepSphere(Query, Contact);
		if (QueryResult != EBallQueryResult::Clear && QueryResult != EBallQueryResult::Contact)
		{ return Finish(EBallStepResult::QueryFailure); }
		if (QueryResult == EBallQueryResult::Contact)
		{
			Output.ContactCount = 1;
			Output.Contacts[0] = Contact;
			return Finish(EBallStepResult::ContactSolverRequired);
		}
		if (!FBallIntegrator::Drift(Candidate, Interval)) { return Finish(EBallStepResult::NumericFailure); }
	}
	++Candidate.SimulationFrame;
	if (!Candidate.IsValid()) { return Finish(EBallStepResult::NumericFailure); }
	State = Candidate;
	return Finish(EBallStepResult::Success);
}
