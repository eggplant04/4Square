#pragma once

#include "Ball/BallHistoryBuffer.h"
#include "Ball/BallSimulator.h"

enum class EBallReplayResult : uint8
{
	Success, InvalidRange, BudgetExceeded, MissingFrame, IncompatibleConfiguration, SimulationFailed
};

struct FOURSQUAREVR_API FBallReplayReport
{
	EBallReplayResult Result = EBallReplayResult::InvalidRange;
	EBallStepResult StepFailure = EBallStepResult::Success;
	uint64 RequestedFrames = 0;
	uint32 ExecutedFrames = 0;
	uint32 Queries = 0;
	uint32 Microintervals = 0;
	double DurationMicroseconds = 0;
};

/** Early feasibility primitive, NOT server adjudication/lag compensation. No mutation of source history. */
class FOURSQUAREVR_API FBallReplayRunner
{
public:
	EBallReplayResult Replay(const FBallHistoryBuffer& History, const FBallIdentity& Identity,
		uint64 FromBoundary, uint64 ToBoundary, const FBallPhysicsConfig& Config,
		const FBallNetworkSettings& Settings, const IBallCollisionWorld& World,
		FBallState& OutState, FBallReplayReport& Report);
	const FBallPerformanceCounters& GetPerformanceCounters() const { return Counters; }

private:
	FBallPerformanceCounters Counters;
};
