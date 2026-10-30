#include "Ball/BallPhysicsConfig.h"

namespace
{
	bool Nonnegative(double Value) { return FMath::IsFinite(Value) && Value >= 0; }
}

bool FBallPhysicsConfig::IsValid() const
{
	return Version > 0 && FMath::IsFinite(Radius) && Radius >= 0.01
		&& FMath::IsFinite(Mass) && Mass >= 0.001 && !Gravity.ContainsNaN()
		&& Nonnegative(LinearDrag) && Nonnegative(AngularDrag)
		&& Nonnegative(Restitution) && Restitution <= 1
		&& Nonnegative(SurfaceFriction) && Nonnegative(SpinFriction) && Nonnegative(RollingResistance)
		&& Nonnegative(MaximumLinearSpeed) && Nonnegative(MaximumAngularSpeed)
		&& Nonnegative(SleepSpeed) && Nonnegative(WakeSpeed) && WakeSpeed >= SleepSpeed
		&& Nonnegative(SleepAngularSpeed) && Nonnegative(SleepDwellSeconds) && Nonnegative(MinimumImpactSpeed)
		&& FMath::IsFinite(TOITolerance) && TOITolerance >= 1.e-9 && TOITolerance <= 1.e-3
		&& ManifoldIterations >= 1 && ManifoldIterations <= 64 && QueryChannel < ECC_MAX
		&& MaximumWorldCandidates >= 1 && MaximumWorldCandidates <= 4096
		&& FMath::IsFinite(CCDTolerance) && CCDTolerance >= 1.e-6
		&& FMath::IsFinite(PenetrationTolerance) && PenetrationTolerance >= 1.e-6
		&& MaximumCollisionIterations >= 1 && MaximumCollisionIterations <= 64
		&& MaximumMicrointervals >= 1 && MaximumMicrointervals <= 64
		&& FMath::IsFinite(FixedFrequency) && FixedFrequency >= 1 && FixedFrequency <= 1000
		&& FMath::IsFinite(GetSolidSphereInertia());
}

int32 FBallNetworkSettings::GetRevisionFrames(double FixedFrequency) const
{
	const double Frames = RevisionWindowSeconds * FixedFrequency;
	if (!FMath::IsFinite(Frames) || Frames < 0 || Frames > 4095) { return INDEX_NONE; }
	// Never shorten compensation by rounding a fractional frame down.
	return FMath::CeilToInt32(Frames);
}

bool FBallNetworkSettings::IsValid(double FixedFrequency) const
{
	const int32 Frames = GetRevisionFrames(FixedFrequency);
	return FMath::IsFinite(FixedFrequency) && FixedFrequency >= 1 && FixedFrequency <= 1000
		&& FMath::IsFinite(RevisionWindowSeconds) && RevisionWindowSeconds >= 0
		&& Frames >= 0 && HistoryCapacity >= 2 && HistoryCapacity <= 4096
		&& MaximumReplayFrames >= 1 && MaximumReplayFrames < HistoryCapacity
		&& MaximumReplayFrames >= Frames && HistoryCapacity >= Frames + 2;
}

bool FBallFinalityPolicy::CalculateEligibleBoundary(uint64 HeadBoundary, const FBallNetworkSettings& Settings,
	double FixedFrequency, uint64& OutBoundary)
{
	if (!Settings.IsValid(FixedFrequency)) { return false; }
	const uint64 Window = static_cast<uint64>(Settings.GetRevisionFrames(FixedFrequency));
	OutBoundary = HeadBoundary > Window ? HeadBoundary - Window : 0;
	return true;
}

bool FBallFinalityPolicy::IsSequenceNewer(uint32 Candidate, uint32 Reference)
{
	const uint32 Distance = Candidate - Reference;
	return Distance != 0 && Distance < 0x80000000u; // Exact half-range is deliberately ambiguous.
}
