#include "Ball/BallDiagnostics.h"

DEFINE_LOG_CATEGORY(LogHandballBall);

void FBallPerformanceCounters::RecordStep(double Microseconds, bool bSuccess, uint32 InQueries, uint32 InMicrointervals)
{
	++Steps;
	FailedSteps += bSuccess ? 0 : 1;
	Queries += InQueries;
	Microintervals += InMicrointervals;
	TotalStepMicroseconds += Microseconds;
	MaximumStepMicroseconds = FMath::Max(MaximumStepMicroseconds, Microseconds);
}

void FBallPerformanceCounters::RecordReplay(double Microseconds, bool bSuccess, uint32 InFrames)
{
	++ReplayAttempts;
	FailedReplays += bSuccess ? 0 : 1;
	ReplayedFrames += InFrames;
	TotalReplayMicroseconds += Microseconds;
	MaximumReplayMicroseconds = FMath::Max(MaximumReplayMicroseconds, Microseconds);
}
