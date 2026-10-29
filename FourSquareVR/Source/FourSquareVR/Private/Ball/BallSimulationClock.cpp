#include "Ball/BallSimulationClock.h"

bool FBallClockSettings::IsValid() const
{
	return FMath::IsFinite(FixedFrequency) && FixedFrequency >= 1 && FixedFrequency <= 1000
		&& MaximumCatchUpSteps > 0 && MaximumCatchUpSteps <= 1024
		&& FMath::IsFinite(MaximumBacklogSeconds) && MaximumBacklogSeconds >= 1.0 / FixedFrequency;
}

bool FBallSimulationClock::Initialize(const FBallClockSettings& InSettings, EBallClockRole InRole, uint64 Frame, uint32 Timebase)
{
	if (!InSettings.IsValid() || Frame == MAX_uint64 || Timebase == 0
		|| static_cast<uint8>(InRole) > static_cast<uint8>(EBallClockRole::PredictedClient)) { return false; }
	Settings = InSettings;
	Role = InRole;
	CurrentFrame = Frame;
	TimebaseVersion = Timebase;
	FixedDeltaTime = 1.0 / Settings.FixedFrequency;
	Accumulator = 0;
	Diagnostics = {};
	Status = EBallClockStatus::Running;
	bInitialized = true;
	return true;
}

bool FBallSimulationClock::BeginRenderFrame(double ElapsedSeconds)
{
	if (!bInitialized || !FMath::IsFinite(ElapsedSeconds) || ElapsedSeconds < 0
		|| !FMath::IsFinite(Accumulator + ElapsedSeconds)) { ++Diagnostics.InvalidDeltas; return false; }
	++Diagnostics.RenderFrames;
	Diagnostics.StepsThisFrame = 0;
	if (Status != EBallClockStatus::Running)
	{
		Diagnostics.ExplicitlyDiscardedSeconds += ElapsedSeconds; // Already in a declared pause/recovery boundary.
		return false;
	}
	Accumulator += ElapsedSeconds;
	Diagnostics.PeakBacklogSeconds = FMath::Max(Diagnostics.PeakBacklogSeconds, Accumulator);
	if (Accumulator > Settings.MaximumBacklogSeconds)
	{
		++Diagnostics.Overruns;
		Status = Role == EBallClockRole::Authority ? EBallClockStatus::AuthorityPaused : EBallClockStatus::NeedsBaseline;
		return false;
	}
	return true;
}

bool FBallSimulationClock::HasStepAvailable() const
{
	// Small, fixed numerical allowance prevents losing a boundary to repeated double addition.
	return bInitialized && Status == EBallClockStatus::Running && CurrentFrame < MAX_uint64 - 1
		&& Diagnostics.StepsThisFrame < Settings.MaximumCatchUpSteps && Accumulator + FixedDeltaTime * 1.e-10 >= FixedDeltaTime;
}

bool FBallSimulationClock::ConsumeCompletedStep()
{
	if (!HasStepAvailable()) { return false; }
	Accumulator = FMath::Max(0.0, Accumulator - FixedDeltaTime);
	++CurrentFrame;
	++Diagnostics.CompletedSteps;
	++Diagnostics.StepsThisFrame;
	return true;
}

bool FBallSimulationClock::ResumeAuthority(uint32 NewTimebase)
{
	if (!bInitialized || Role != EBallClockRole::Authority || Status != EBallClockStatus::AuthorityPaused
		|| NewTimebase <= TimebaseVersion) { return false; }
	Diagnostics.ExplicitlyDiscardedSeconds += Accumulator;
	Accumulator = 0;
	TimebaseVersion = NewTimebase;
	Diagnostics.StepsThisFrame = 0;
	Status = EBallClockStatus::Running;
	return true;
}

bool FBallSimulationClock::ApplyClientBaseline(uint64 Frame, uint32 NewTimebase)
{
	if (!bInitialized || Role != EBallClockRole::PredictedClient || Frame == MAX_uint64
		|| NewTimebase < TimebaseVersion || NewTimebase == 0) { return false; }
	Diagnostics.ExplicitlyDiscardedSeconds += Accumulator;
	Accumulator = 0;
	CurrentFrame = Frame;
	TimebaseVersion = NewTimebase;
	Diagnostics.StepsThisFrame = 0;
	Status = EBallClockStatus::Running;
	return true;
}
