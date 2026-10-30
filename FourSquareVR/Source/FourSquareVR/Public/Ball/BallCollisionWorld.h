#pragma once

#include "Ball/BallPhysicsTypes.h"

struct FOURSQUAREVR_API FBallCollisionQuery
{
	FVector Start = FVector::ZeroVector;
	FVector End = FVector::ZeroVector;
	double Radius = 0;
	FBallFrameTime Time;
	double EndFraction = 1;
	double TOIFractionTolerance = 1.e-6;
	double SpatialTolerance = 0.01;
	double ProbeDistance = 0;
	bool bProximity = false;
	TEnumAsByte<ECollisionChannel> Channel = ECC_WorldDynamic;
	FName RequiredProfile = NAME_None;
	bool bTraceComplex = false;
	bool bUsePhysicalMaterials = false;
	int32 MaximumCandidates = 64;
};

enum class EBallQueryResult : uint8 { Clear, Contact, InvalidQuery, CapacityExceeded };

struct FOURSQUAREVR_API FBallContactBatch
{
	int32 Count = 0;
	FBallContact Contacts[BallLimits::Contacts];
	/** Keep all contacts tied with the earliest TOI, in strict stable order. */
	bool Add(const FBallContact& Contact, const FBallCollisionQuery& Query);
	void Sort();
};

/** Queries return spatial facts, never impulses. Revision/time are explicit replay dependencies. */
class FOURSQUAREVR_API IBallCollisionWorld
{
public:
	virtual ~IBallCollisionWorld() = default;
	virtual uint32 GetRevision() const = 0;
	virtual EBallQueryResult SweepSphere(const FBallCollisionQuery& Query, FBallContact& OutContact) const = 0;
	virtual EBallQueryResult GatherContacts(const FBallCollisionQuery& Query, FBallContactBatch& Output) const
	{
		Output.Count = 0;
		FBallContact Contact;
		const EBallQueryResult Result = SweepSphere(Query, Contact);
		if (Result == EBallQueryResult::Contact) { Output.Contacts[0] = Contact; Output.Count = 1; }
		return Result;
	}
};
