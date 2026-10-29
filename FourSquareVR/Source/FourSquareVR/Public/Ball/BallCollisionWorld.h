#pragma once

#include "Ball/BallPhysicsTypes.h"

struct FOURSQUAREVR_API FBallCollisionQuery
{
	FVector Start = FVector::ZeroVector;
	FVector End = FVector::ZeroVector;
	double Radius = 0;
	FBallFrameTime Time;
	double EndFraction = 1;
};

enum class EBallQueryResult : uint8 { Clear, Contact, InvalidQuery };

/** Queries return spatial facts, never impulses. Revision/time are explicit replay dependencies. */
class FOURSQUAREVR_API IBallCollisionWorld
{
public:
	virtual ~IBallCollisionWorld() = default;
	virtual uint32 GetRevision() const = 0;
	virtual EBallQueryResult SweepSphere(const FBallCollisionQuery& Query, FBallContact& OutContact) const = 0;
};
