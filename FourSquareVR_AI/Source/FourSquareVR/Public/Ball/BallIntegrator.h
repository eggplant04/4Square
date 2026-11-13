#pragma once

#include "Ball/BallPhysicsConfig.h"
#include "Ball/BallPhysicsTypes.h"

struct FOURSQUAREVR_API FBallIntegrator
{
	static bool ApplyVelocityLimits(FBallState &State, const FBallPhysicsConfig &Config);
	static bool Kick(FBallState &State, const FVector &LinearAcceleration, const FVector &AngularAcceleration,
					 const FBallPhysicsConfig &Config, double DeltaTime);
	static bool Drift(FBallState &State, double DeltaTime);
};
