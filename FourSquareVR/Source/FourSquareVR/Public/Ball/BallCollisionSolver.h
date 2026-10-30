#pragma once

#include "Ball/BallCollisionWorld.h"

/** Pure geometry/response. No UWorld, Actor, tracking, clock, or presentation dependencies. */
struct FOURSQUAREVR_API FBallCollisionSolver
{
	static bool SphereTOI(const FVector& RelativeStart, const FVector& RelativeDelta, double Radius, double& OutTime);
	static bool SweepBox(const FBallCollisionQuery& Query, const FVector& Minimum, const FVector& Maximum, FBallContact& Contact);
	static bool SweepSphere(const FBallCollisionQuery& Query, const FVector& Center, double Radius, FBallContact& Contact);
	static EBallStepResult Resolve(FBallState& State, const FBallContactBatch& Contacts,
		const FBallPhysicsConfig& Config, double FrameFraction, FBallStepOutput& Output);
	static bool CanSupport(const FBallContactBatch& Contacts, const FVector& Acceleration,
		const FBallPhysicsConfig& Config, FVector& OutNormal, FBallColliderId& OutCollider);
	static bool RefreshContacts(FBallState& State, const FBallContactBatch& Contacts, bool bSupporting);
	static bool Separate(FBallState& State, const FBallContactBatch& Contacts, double Skin);
};
