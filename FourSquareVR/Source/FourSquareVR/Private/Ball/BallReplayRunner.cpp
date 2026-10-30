#include "Ball/BallReplayRunner.h"
#include "Ball/BallMotionHistory.h"
#include "HAL/PlatformTime.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

EBallReplayResult FBallReplayRunner::Replay(const FBallHistoryBuffer &History, const FBallIdentity &Identity,
											uint64 FromBoundary, uint64 ToBoundary, const FBallPhysicsConfig &Config,
											const FBallNetworkSettings &Settings, const IBallCollisionWorld &World,
											FBallState &OutState, FBallReplayReport &Report)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Handball_BoundedReplay);
	const double Start = FPlatformTime::Seconds();
	Report = {};
	const auto Finish = [&](EBallReplayResult Result) {
		Report.Result = Result;
		Report.DurationMicroseconds = (FPlatformTime::Seconds() - Start) * 1.e6;
		Counters.RecordReplay(Report.DurationMicroseconds, Result == EBallReplayResult::Success, Report.ExecutedFrames);
		return Result;
	};
	if (!Identity.IsValid() || ToBoundary < FromBoundary || ToBoundary == MAX_uint64)
	{
		return Finish(EBallReplayResult::InvalidRange);
	}
	Report.RequestedFrames = ToBoundary - FromBoundary;
	if (!Config.IsValid() || !Settings.IsValid(Config.FixedFrequency))
	{
		return Finish(EBallReplayResult::IncompatibleConfiguration);
	}
	if (Report.RequestedFrames > static_cast<uint64>(Settings.MaximumReplayFrames))
	{
		return Finish(EBallReplayResult::BudgetExceeded);
	}
	FBallHistoryFrame Record;
	if (!History.CopyFrame(FromBoundary, Identity, Record))
	{
		return Finish(EBallReplayResult::MissingFrame);
	}
	FBallState Candidate = Record.State;
	if (Candidate.ConfigVersion != static_cast<uint32>(Config.Version) ||
		Candidate.EnvironmentRevision != World.GetRevision())
	{
		return Finish(EBallReplayResult::IncompatibleConfiguration);
	}
	// Preflight the entire bounded dependency range before starting any numerical work.
	for (uint64 Frame = FromBoundary; Frame < ToBoundary; ++Frame)
	{
		if (!History.CopyFrame(Frame, Identity, Record))
		{
			return Finish(EBallReplayResult::MissingFrame);
		}
		if (Record.State.ConfigVersion != static_cast<uint32>(Config.Version) ||
			Record.State.EnvironmentRevision != World.GetRevision() ||
			Record.State.MaterialVersion != Candidate.MaterialVersion || !World.ValidateInputs(Record.Inputs))
		{
			return Finish(EBallReplayResult::IncompatibleConfiguration);
		}
		const auto *Motion = World.GetMotionFrame(Record.Inputs);
		if (Motion &&
			(Motion->Epoch != Identity.Epoch || FMath::Abs(Motion->DeltaTime - Config.GetFixedDeltaTime()) > 1.e-12))
		{
			return Finish(EBallReplayResult::IncompatibleConfiguration);
		}
	}
	FBallSimulator Simulator;
	FBallStepOutput Output;
	for (uint64 Frame = FromBoundary; Frame < ToBoundary; ++Frame)
	{
		if (!History.CopyFrame(Frame, Identity, Record))
		{
			return Finish(EBallReplayResult::MissingFrame);
		}
		Report.StepFailure = Simulator.SimulateStep(Candidate, Record.Inputs, Config, World, Output);
		Report.Queries += Output.Diagnostics.Queries;
		Report.Microintervals += Output.Diagnostics.Microintervals;
		if (Report.StepFailure != EBallStepResult::Success)
		{
			return Finish(EBallReplayResult::SimulationFailed);
		}
		++Report.ExecutedFrames;
	}
	OutState = Candidate; // Only complete replay commits to the destination.
	return Finish(EBallReplayResult::Success);
}
