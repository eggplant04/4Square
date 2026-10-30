#pragma once

#include "CoreMinimal.h"

enum class EBallClockRole : uint8 { Authority, PredictedClient };
enum class EBallClockStatus : uint8 { Running, AuthorityPaused, NeedsBaseline };

struct FOURSQUAREVR_API FBallClockSettings
{
	double FixedFrequency = 120;
	int32 MaximumCatchUpSteps = 8;
	double MaximumBacklogSeconds = 0.25;
	bool IsValid() const;
};

struct FOURSQUAREVR_API FBallClockDiagnostics
{
	uint64 RenderFrames = 0;
	uint64 CompletedSteps = 0;
	uint64 Overruns = 0;
	uint64 InvalidDeltas = 0;
	int32 StepsThisFrame = 0;
	double PeakBacklogSeconds = 0;
	double ExplicitlyDiscardedSeconds = 0;
};

/** Scheduling only. Consume a step only AFTER the corresponding simulator transaction succeeds. */
class FOURSQUAREVR_API FBallSimulationClock
{
public:
	bool Initialize(const FBallClockSettings& InSettings, EBallClockRole InRole, uint64 Frame = 0, uint32 Timebase = 1);
	bool BeginRenderFrame(double ElapsedSeconds);
	bool HasStepAvailable() const;
	bool ConsumeCompletedStep();
	bool PauseAuthority();
	/** Explicit authority pause boundary: no frame advance; discarded debt is counted and timebase must increase. */
	bool ResumeAuthority(uint32 NewTimebase);
	/** Recovery does not pretend to have simulated skipped frames. Caller supplies an actual baseline. */
	bool ApplyClientBaseline(uint64 Frame, uint32 NewTimebase);
	uint64 GetFrame() const { return CurrentFrame; }
	uint32 GetTimebase() const { return TimebaseVersion; }
	double GetFixedDeltaTime() const { return FixedDeltaTime; }
	double GetAccumulatorSeconds() const { return Accumulator; }
	EBallClockStatus GetStatus() const { return Status; }
	const FBallClockDiagnostics& GetDiagnostics() const { return Diagnostics; }

private:
	FBallClockSettings Settings;
	FBallClockDiagnostics Diagnostics;
	EBallClockRole Role = EBallClockRole::Authority;
	EBallClockStatus Status = EBallClockStatus::AuthorityPaused;
	uint64 CurrentFrame = 0;
	uint32 TimebaseVersion = 0;
	double FixedDeltaTime = 1.0 / 120.0;
	double Accumulator = 0;
	bool bInitialized = false;
};
