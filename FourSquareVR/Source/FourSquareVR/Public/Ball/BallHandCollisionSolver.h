#pragma once
#include "Ball/BallMotionHistory.h"
#include "Ball/BallCollisionWorld.h"

/** Pure relative-motion spatial calculations, reusable for palms and recorded kinematics. */
struct FOURSQUAREVR_API FBallHandCollisionSolver
{
	static const FBallMotionSegment *SegmentAt(const FBallMotionFrame &Frame, const FBallColliderMotion &Collider,
											   double Fraction);
	static FVector CenterAt(const FBallMotionSegment &Segment, double Fraction);
	static double Separation(const FBallMotionFrame &Frame, const FBallColliderMotion &Collider, double Fraction,
							 const FVector &Ball, double Radius);
	static bool Sweep(const FBallMotionFrame &Frame, const FBallColliderMotion &Collider,
					  const FBallCollisionQuery &Query, FBallContact &Contact);
	static double NextBoundary(const FBallMotionFrame &Frame, double Fraction, double End);
	static bool CouldReach(const FBallMotionFrame &Frame, const FVector &Ball, double Radius);
	static double MaximumRelativeTravel(const FBallMotionFrame &Frame, const FBallCollisionQuery &Query);
};
