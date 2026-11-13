#pragma once

#include "Ball/BallCollisionWorld.h"

class UWorld;
class ULevel;
class UPrimitiveComponent;
class UBodySetup;
class UPhysicalMaterial;
class AActor;

struct FOURSQUAREVR_API FBallWorldQueryStatistics
{
	uint64 Queries = 0;
	uint64 NarrowphaseQueries = 0;
	uint32 PeakCandidates = 0;
};

/** Immutable static-geometry epoch. Does not rewind Actors or query moving geometry as historical data. */
class FOURSQUAREVR_API FUnrealBallCollisionWorld final : public IBallCollisionWorld
{
public:
	FUnrealBallCollisionWorld();
	virtual ~FUnrealBallCollisionWorld() override;
	FUnrealBallCollisionWorld(const FUnrealBallCollisionWorld&) = delete;
	FUnrealBallCollisionWorld& operator=(const FUnrealBallCollisionWorld&) = delete;
	bool Initialize(UWorld* World, uint32 Revision);
	virtual uint32 GetRevision() const override;
	virtual EBallQueryResult SweepSphere(const FBallCollisionQuery& Query, FBallContact& Output) const override;
	virtual EBallQueryResult GatherContacts(const FBallCollisionQuery& Query, FBallContactBatch& Output) const override;
	void Invalidate() { bDirty = true; }
	SIZE_T GetAllocatedBytes() const;
	int32 NumComponents() const { return Records.Num(); }
	const FBallWorldQueryStatistics& GetStatistics() const { return Statistics; }

private:
	struct FStaticRecord
	{
		TWeakObjectPtr<UPrimitiveComponent> Component;
		TWeakObjectPtr<UBodySetup> Body;
		TWeakObjectPtr<UPhysicalMaterial> PhysicalMaterial;
		FGuid GeometryGuid;
		FTransform Transform;
		FBox Bounds;
		FCollisionResponseContainer Responses;
		ECollisionEnabled::Type Enabled = ECollisionEnabled::NoCollision;
		ECollisionChannel ObjectType = ECC_WorldStatic;
		FName Profile;
		FBallColliderId Id;
		FString Key;
		FBallSurfaceMaterial Material;
		int32 OwnerComponentCount = 0;
	};
	TWeakObjectPtr<UWorld> BoundWorld;
	TArray<FStaticRecord> Records;
	TMap<TWeakObjectPtr<UPrimitiveComponent>, int32> Lookup;
	mutable TArray<FHitResult> ScratchHits;
	mutable TArray<TWeakObjectPtr<AActor>> PendingActors;
	FDelegateHandle SpawnHandle, AddedLevelHandle, RemovedLevelHandle;
	uint32 Revision = 0;
	mutable bool bDirty = false;
	mutable FBallWorldQueryStatistics Statistics;
	void Detach();
	void OnSpawn(AActor* Actor);
	void OnLevel(ULevel* Level, UWorld* World);
	bool Unchanged() const;
	static bool Eligible(UPrimitiveComponent* Component);
};
