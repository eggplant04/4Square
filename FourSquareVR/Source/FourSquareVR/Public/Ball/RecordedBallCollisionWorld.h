#pragma once

#include "Ball/BallCollisionWorld.h"

/** Immutable-after-setup analytic half-spaces for reproducible query/replay fixtures, not a UWorld adapter. */
class FOURSQUAREVR_API FRecordedBallCollisionWorld final : public IBallCollisionWorld
{
public:
	explicit FRecordedBallCollisionWorld(uint32 Revision = 1);
	bool AddPlane(FBallColliderId Id, const FVector& Point, const FVector& OutwardNormal);
	virtual uint32 GetRevision() const override { return WorldRevision; }
	virtual EBallQueryResult SweepSphere(const FBallCollisionQuery& Query, FBallContact& OutContact) const override;

private:
	struct FPlaneRecord { FBallColliderId Id; FVector Point; FVector Normal; };
	TArray<FPlaneRecord> Planes;
	uint32 WorldRevision;
	mutable bool bSealed = false;
};
