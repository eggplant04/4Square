#pragma once

#include "Ball/BallCollisionWorld.h"

/** Immutable-after-setup analytic half-spaces for reproducible query/replay fixtures, not a UWorld adapter. */
class FOURSQUAREVR_API FRecordedBallCollisionWorld final : public IBallCollisionWorld
{
public:
	explicit FRecordedBallCollisionWorld(uint32 Revision = 1);
	bool AddPlane(FBallColliderId Id, const FVector& Point, const FVector& OutwardNormal, FBallSurfaceMaterial Material = {});
	bool AddSphere(FBallColliderId Id, const FVector& Center, double Radius, FBallSurfaceMaterial Material = {});
	bool AddBox(FBallColliderId Id, const FVector& Minimum, const FVector& Maximum, FBallSurfaceMaterial Material = {});
	virtual uint32 GetRevision() const override { return WorldRevision; }
	virtual EBallQueryResult SweepSphere(const FBallCollisionQuery& Query, FBallContact& OutContact) const override;
	virtual EBallQueryResult GatherContacts(const FBallCollisionQuery& Query, FBallContactBatch& Output) const override;

private:
	struct FShapeRecord { FBallColliderId Id; FVector A; FVector B; double Radius = 0; uint8 Shape = 0; FBallSurfaceMaterial Material; };
	bool Add(const FShapeRecord& Record);
	TArray<FShapeRecord> Shapes;
	uint32 WorldRevision;
	mutable bool bSealed = false;
};
