#pragma once
#include "Ball/BallMotionHistory.h"
#include "Ball/BallCollisionWorld.h"

struct FOURSQUAREVR_API FBallPalmDistance
{
	double Separation = 0;
	FVector Normal = FVector::UpVector, Point = FVector::ZeroVector;
	uint32 Feature = 0;
};

/** True core-box + round-radius geometry. No UObject queries or hidden solver state. */
struct FOURSQUAREVR_API FBallPalmGeometry
{
	static FVector RotationVector(const FQuat &A, const FQuat &B);
	static FQuat InterpolateOrientation(const FQuat &A, const FQuat &B, double Alpha);
	static FQuat OrientationAt(const FBallMotionSegment &S, double Fraction);
	static FVector AngularDisplacement(const FBallMotionSegment &S);
	static FVector CenterAt(const FBallColliderMotion &C, const FBallMotionSegment &S, double Fraction);
	static FVector SurfaceVelocity(const FBallColliderMotion &C, const FBallMotionSegment &S, double Fraction,
								   double Dt, const FVector &Point);
	static FBallPalmDistance Distance(const FVector &Ball, double BallRadius, const FVector &Center,
									  const FQuat &Orientation, const FVector &HalfExtents, double RoundRadius);
	static EBallQueryResult Sweep(const FBallMotionFrame &F, const FBallColliderMotion &C, const FBallMotionSegment &S,
								  const FBallCollisionQuery &Q, FBallContact &Contact);
};
