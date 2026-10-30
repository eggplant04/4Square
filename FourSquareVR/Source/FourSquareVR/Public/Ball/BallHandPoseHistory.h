#pragma once
#include "Ball/BallMotionHistory.h"

/** Bounded observations and pure timestamp reconstruction. Never reads component velocity or current transforms during
 * replay. */
class FOURSQUAREVR_API FBallHandPoseHistory
{
  public:
	bool Initialize(int32 Capacity = 256);
	EBallTrackingRejection Add(const FBallHandObservation &Pose, const FBallHandTrackingSettings &Settings);
	void NotifyDiscontinuity();
	const FBallHandSample *GetLatest() const;
	bool BuildMotion(double StartSeconds, const FBallPalmSettings &Palm, const FBallHandTrackingSettings &Settings,
					 uint32 Participant, EBallHandSide Side, FBallMotionFrame &Frame, EBallTrackingRejection &OutReason,
					 uint32 ConfigurationVersion = 1) const;
	SIZE_T GetAllocatedBytes() const
	{
		return Samples.GetAllocatedSize();
	}

  private:
	void BreakContinuity();
	void Push(const FBallHandSample &Sample);
	const FBallHandSample &At(int32 Index) const;
	TArray<FBallHandSample> Samples;
	int32 Head = 0, Count = 0;
	uint32 Generation = 1;
	bool bNeedsWarmup = true;
};
