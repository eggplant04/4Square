#pragma once

#include "CoreMinimal.h"

DECLARE_LOG_CATEGORY_EXTERN(LogHandballBall, Log, All);

/** Fixed-size aggregation suitable for the simulation hot path; units are microseconds/bytes. */
struct FOURSQUAREVR_API FBallPerformanceCounters
{
	uint64 Steps = 0;
	uint64 FailedSteps = 0;
	uint64 Queries = 0;
	uint64 Microintervals = 0;
	uint64 ReplayAttempts = 0;
	uint64 FailedReplays = 0;
	uint64 ReplayedFrames = 0;
	double TotalStepMicroseconds = 0;
	double MaximumStepMicroseconds = 0;
	double TotalReplayMicroseconds = 0;
	double MaximumReplayMicroseconds = 0;

	void RecordStep(double Microseconds, bool bSuccess, uint32 InQueries, uint32 InMicrointervals);
	void RecordReplay(double Microseconds, bool bSuccess, uint32 InFrames);
};
