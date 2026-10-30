#pragma once
#include "Ball/BallMotionHistory.h"
#include "Ball/BallCollisionWorld.h"

/** Explicit checkpointed episode policy, independent of live tracking and networking. */
struct FOURSQUAREVR_API FBallMotionContactPolicy
{
	static void RetireAbsent(FBallState &State, const FBallMotionFrame &Frame);
	static uint16 PrepareQuery(FBallState &State, const FBallMotionFrame &Frame, double Fraction, double Radius,
							   double ReleaseDistance);
	static void FilterConflicts(FBallState &State, FBallContactBatch &Batch, double Skin, FBallStepOutput &Output);
	static void CompleteResponse(FBallState &State, const FBallContactBatch &Batch, int32 FirstEvent,
								 FBallStepOutput &Output);
};
