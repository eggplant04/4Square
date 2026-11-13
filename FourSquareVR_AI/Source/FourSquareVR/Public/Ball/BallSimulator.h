#pragma once

#include "Ball/BallCollisionWorld.h"
#include "Ball/BallDiagnostics.h"
#include "Ball/BallPhysicsConfig.h"

/** Stage 2 free-flight foundation. Colliding/resting/contact-manifold steps fail explicitly until Stage 3. */
class FOURSQUAREVR_API FBallSimulator
{
public:
	EBallStepResult SimulateStep(FBallState& State, const FBallStepInputs& Inputs,
		const FBallPhysicsConfig& Config, const IBallCollisionWorld& World, FBallStepOutput& Output);
	const FBallPerformanceCounters& GetPerformanceCounters() const { return Counters; }

private:
	FBallPerformanceCounters Counters;
};
