#include "Ball/UnrealBallCollisionWorld.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "PhysicsEngine/BodySetup.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicalMaterials/PhysicalMaterial.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

namespace
{
FString CanonicalKey(UPrimitiveComponent *Component)
{
	FString Key = Component->GetPathName();
	int32 Start;
	while ((Start = Key.Find(TEXT("UEDPIE_"))) != INDEX_NONE)
	{
		int32 End = Start + 7;
		while (End < Key.Len() && FChar::IsDigit(Key[End]))
		{
			++End;
		}
		if (End < Key.Len() && Key[End] == '_')
		{
			Key.RemoveAt(Start, End - Start + 1);
		}
		else
		{
			break;
		}
	}
	return Key;
}
uint64 StableId(const FString &Key)
{
	FTCHARToUTF8 UTF8(*Key);
	uint64 Hash = 14695981039346656037ull;
	for (int32 I = 0; I < UTF8.Length(); ++I)
	{
		Hash = (Hash ^ static_cast<uint8>(UTF8.Get()[I])) * 1099511628211ull;
	}
	Hash &= 0x3fffffffffffffffull; // Upper namespaces are reserved for recorded hands/kinematics.
	return Hash == 0 ? 1 : Hash;
}
uint32 Feature(const FVector &N, int32 Face)
{
	if (Face >= 0)
	{
		return static_cast<uint32>(Face);
	}
	const int32 Values[] = {FMath::RoundToInt(N.X * 10000), FMath::RoundToInt(N.Y * 10000),
							FMath::RoundToInt(N.Z * 10000)};
	uint32 Hash = 2166136261u;
	for (const int32 Value : Values)
	{
		Hash = (Hash ^ static_cast<uint32>(Value)) * 16777619u;
	}
	return Hash;
}
FBallSurfaceMaterial ReadMaterial(UPhysicalMaterial *Material)
{
	FBallSurfaceMaterial Result;
	if (Material)
	{
		Result.bOverride = true;
		Result.Restitution = FMath::Clamp(static_cast<double>(Material->Restitution), 0.0, 1.0);
		Result.Friction = FMath::Max(0.0, static_cast<double>(Material->Friction));
	}
	return Result;
}
} // namespace

FUnrealBallCollisionWorld::FUnrealBallCollisionWorld()
{
	Records.Reserve(256);
	PendingActors.Reserve(64);
	ScratchHits.Reserve(256);
}
FUnrealBallCollisionWorld::~FUnrealBallCollisionWorld()
{
	Detach();
}

void FUnrealBallCollisionWorld::Detach()
{
	if (auto *World = BoundWorld.Get())
	{
		World->RemoveOnActorSpawnedHandler(SpawnHandle);
	}
	FWorldDelegates::LevelAddedToWorld.Remove(AddedLevelHandle);
	FWorldDelegates::LevelRemovedFromWorld.Remove(RemovedLevelHandle);
	BoundWorld.Reset();
}

bool FUnrealBallCollisionWorld::Eligible(UPrimitiveComponent *C)
{
	return IsValid(C) && C->IsRegistered() && C->Mobility == EComponentMobility::Static &&
		   C->IsQueryCollisionEnabled() && !C->IsSimulatingPhysics() && !C->ComponentHasTag(TEXT("BallIgnore")) &&
		   C->GetBodySetup();
}

bool FUnrealBallCollisionWorld::Initialize(UWorld *World, uint32 InRevision)
{
	Detach();
	Records.Reset();
	Lookup.Reset();
	PendingActors.Reset();
	Revision = InRevision;
	bDirty = false;
	Statistics = {};
	if (!World || Revision == 0 || !IsInGameThread())
	{
		return false;
	}
	BoundWorld = World;
	for (TActorIterator<AActor> It(World); It; ++It)
	{
		TInlineComponentArray<UPrimitiveComponent *> Components(*It);
		for (UPrimitiveComponent *C : Components)
		{
			if (!Eligible(C))
			{
				continue;
			}
			if (Records.Num() == 4096)
			{
				bDirty = true;
				return false;
			}
			FStaticRecord R;
			R.Component = C;
			R.Body = C->GetBodySetup();
			R.GeometryGuid = C->GetBodySetup()->BodySetupGuid;
			R.Transform = C->GetComponentTransform();
			R.Bounds = C->Bounds.GetBox();
			R.Responses = C->GetCollisionResponseToChannels();
			R.Enabled = C->GetCollisionEnabled();
			R.Profile = C->GetCollisionProfileName();
			R.ObjectType = C->GetCollisionObjectType();
			R.OwnerComponentCount = C->GetOwner()->GetComponents().Num();
			if (const FBodyInstance *Body = C->GetBodyInstance())
			{
				R.PhysicalMaterial = Body->GetSimplePhysicalMaterial();
			}
			R.Material = ReadMaterial(R.PhysicalMaterial.Get());
			R.Key = CanonicalKey(C);
			R.Id = {StableId(R.Key), Revision};
			Records.Add(MoveTemp(R));
		}
	}
	Records.Sort([](const FStaticRecord &A, const FStaticRecord &B) { return A.Key < B.Key; });
	for (int32 I = 0; I < Records.Num(); ++I)
		for (int32 J = 0; J < I; ++J)
		{
			if (Records[I].Id == Records[J].Id)
			{
				bDirty = true;
				return false;
			}
		}
	for (int32 I = 0; I < Records.Num(); ++I)
	{
		Lookup.Add(Records[I].Component, I);
	}
	SpawnHandle = World->AddOnActorSpawnedHandler(
		FOnActorSpawned::FDelegate::CreateRaw(this, &FUnrealBallCollisionWorld::OnSpawn));
	AddedLevelHandle = FWorldDelegates::LevelAddedToWorld.AddRaw(this, &FUnrealBallCollisionWorld::OnLevel);
	RemovedLevelHandle = FWorldDelegates::LevelRemovedFromWorld.AddRaw(this, &FUnrealBallCollisionWorld::OnLevel);
	return true;
}

void FUnrealBallCollisionWorld::OnSpawn(AActor *Actor)
{
	if (PendingActors.Num() == 64)
	{
		bDirty = true;
	}
	else
	{
		PendingActors.Add(Actor);
	}
}
void FUnrealBallCollisionWorld::OnLevel(ULevel *Level, UWorld *World)
{
	if (World == BoundWorld.Get())
	{
		bDirty = true;
	}
}

bool FUnrealBallCollisionWorld::Unchanged() const
{
	if (bDirty || !BoundWorld.IsValid() || !IsInGameThread())
	{
		return false;
	}
	for (const auto &Pending : PendingActors)
	{
		if (AActor *Actor = Pending.Get())
		{
			TInlineComponentArray<UPrimitiveComponent *> Components(Actor);
			for (auto *C : Components)
			{
				if (Eligible(C))
				{
					bDirty = true;
					return false;
				}
			}
		}
	}
	PendingActors.Reset();
	for (const auto &R : Records)
	{
		auto *C = R.Component.Get();
		const FBallSurfaceMaterial Material = ReadMaterial(R.PhysicalMaterial.Get());
		if (!Eligible(C) || C->GetBodySetup() != R.Body.Get() || C->GetBodySetup()->BodySetupGuid != R.GeometryGuid ||
			!C->GetComponentTransform().Equals(R.Transform, 1.e-6) || !C->Bounds.GetBox().Equals(R.Bounds, 1.e-6) ||
			C->GetCollisionEnabled() != R.Enabled || C->GetCollisionProfileName() != R.Profile ||
			C->GetCollisionObjectType() != R.ObjectType || !(C->GetCollisionResponseToChannels() == R.Responses) ||
			C->GetOwner()->GetComponents().Num() != R.OwnerComponentCount ||
			Material.Restitution != R.Material.Restitution || Material.Friction != R.Material.Friction ||
			(C->GetBodyInstance() && C->GetBodyInstance()->GetSimplePhysicalMaterial() != R.PhysicalMaterial.Get()))
		{
			bDirty = true;
			return false;
		}
	}
	return true;
}

uint32 FUnrealBallCollisionWorld::GetRevision() const
{
	return Unchanged() ? Revision : 0;
}
SIZE_T FUnrealBallCollisionWorld::GetAllocatedBytes() const
{
	SIZE_T Bytes = sizeof(*this) + Records.GetAllocatedSize() + Lookup.GetAllocatedSize() +
				   PendingActors.GetAllocatedSize() + ScratchHits.GetAllocatedSize();
	for (const auto &R : Records)
	{
		Bytes += R.Key.GetAllocatedSize();
	}
	return Bytes;
}

EBallQueryResult FUnrealBallCollisionWorld::SweepSphere(const FBallCollisionQuery &Query, FBallContact &Output) const
{
	FBallContactBatch Batch;
	const auto Result = GatherContacts(Query, Batch);
	if (Result == EBallQueryResult::Contact)
	{
		Output = Batch.Contacts[0];
	}
	return Result;
}

EBallQueryResult FUnrealBallCollisionWorld::GatherContacts(const FBallCollisionQuery &Q,
														   FBallContactBatch &Output) const
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Handball_StaticWorldQuery);
	Output.Count = 0;
	++Statistics.Queries;
	if (!Unchanged() || Q.Start.ContainsNaN() || Q.End.ContainsNaN() || !Q.Time.IsValid() ||
		!FMath::IsFinite(Q.Radius) || Q.Radius <= 0 || Q.Radius > 1.e6 || Q.ProbeDistance < 0 ||
		!FMath::IsFinite(Q.ProbeDistance) || Q.MaximumCandidates < 1)
	{
		return EBallQueryResult::InvalidQuery;
	}
	FBox Bounds(ForceInit);
	Bounds += Q.Start;
	Bounds += Q.End;
	Bounds = Bounds.ExpandBy(Q.Radius + Q.ProbeDistance + Q.SpatialTolerance);
	uint32 Candidates = 0;
	for (const auto &R : Records)
	{
		if (Bounds.Intersect(R.Bounds) && R.Responses.GetResponse(Q.Channel) == ECR_Block &&
			(Q.RequiredProfile.IsNone() || R.Profile == Q.RequiredProfile))
		{
			++Candidates;
		}
	}
	Statistics.PeakCandidates = FMath::Max(Statistics.PeakCandidates, Candidates);
	if (Candidates > static_cast<uint32>(Q.MaximumCandidates))
	{
		return EBallQueryResult::CapacityExceeded;
	}
	FCollisionQueryParams Params(SCENE_QUERY_STAT(HandballStaticSweep), Q.bTraceComplex);
	Params.bFindInitialOverlaps = true;
	Params.bReturnFaceIndex = true;
	Params.MobilityType = EQueryMobilityType::Static;
	const double QueryRadius = Q.Radius + (Q.bProximity ? Q.ProbeDistance : 0);
	ScratchHits.Reset();
	++Statistics.NarrowphaseQueries;
	// Object MULTI queries use touch prefiltering in UE 5.6.1: no first-blocker truncation.
	// Engine-managed query scratch may allocate internally; excess results fail explicitly.
	BoundWorld->SweepMultiByObjectType(ScratchHits, Q.Start, Q.bProximity ? Q.Start : Q.End, FQuat::Identity,
									   FCollisionObjectQueryParams(FCollisionObjectQueryParams::AllObjects),
									   FCollisionShape::MakeSphere(QueryRadius), Params);
	if (ScratchHits.Num() > 256)
	{
		return EBallQueryResult::CapacityExceeded;
	}
	double Earliest = 2;
	// Find the true earliest TOI before applying the eight-normal cap.
	for (const auto &Hit : ScratchHits)
	{
		auto *C = Hit.GetComponent();
		const int32 *Index = Lookup.Find(C);
		if (!Index)
		{
			if (Eligible(C))
			{
				bDirty = true;
				return EBallQueryResult::InvalidQuery;
			}
			continue;
		}
		const auto &R = Records[*Index];
		if (R.Responses.GetResponse(Q.Channel) != ECR_Block ||
			(!Q.RequiredProfile.IsNone() && R.Profile != Q.RequiredProfile))
		{
			continue;
		}
		const FVector N = FVector(Hit.Normal).GetSafeNormal();
		if (!Q.bProximity && Hit.PenetrationDepth <= Q.SpatialTolerance &&
			FVector::DotProduct(Q.End - Q.Start, N) >= -1.e-9)
		{
			continue;
		}
		Earliest = FMath::Min(Earliest, static_cast<double>(Hit.Time));
	}
	for (const auto &Hit : ScratchHits)
	{
		auto *C = Hit.GetComponent();
		const int32 *Index = Lookup.Find(C);
		if (!Index)
		{
			continue;
		}
		const auto &R = Records[*Index];
		if (R.Responses.GetResponse(Q.Channel) != ECR_Block ||
			(!Q.RequiredProfile.IsNone() && R.Profile != Q.RequiredProfile))
		{
			continue;
		}
		if (!Q.bProximity && Hit.Time > Earliest + Q.TOIFractionTolerance)
		{
			continue;
		}
		FBallContact Contact;
		Contact.Collider = R.Id;
		Contact.Material = R.Material;
		Contact.Material.bOverride &= Q.bUsePhysicalMaterials;
		Contact.TimeOfImpact = Q.bProximity ? 0 : Hit.Time;
		Contact.Normal = FVector(Hit.Normal).GetSafeNormal();
		Contact.Position = Hit.ImpactPoint;
		if (Hit.bStartPenetrating)
		{
			// World sweep MTD is per shape, rather than a single combined component MTD.
			double Depth = Hit.PenetrationDepth;
			if (!Contact.Normal.IsUnit(1.e-4) || Depth <= 0)
			{
				FMTDResult MTD;
				if (!C->ComputePenetration(MTD, FCollisionShape::MakeSphere(QueryRadius), Q.Start, FQuat::Identity))
				{
					return EBallQueryResult::InvalidQuery;
				}
				Contact.Normal = MTD.Direction.GetSafeNormal();
				Depth = MTD.Distance;
			}
			Contact.Separation = (Q.bProximity ? Q.ProbeDistance : 0) - Depth;
			Contact.PenetrationDepth = FMath::Max(0.0, -Contact.Separation);
			Contact.Position = Q.Start - Contact.Normal * (Q.Radius + Contact.Separation);
		}
		else if (Q.bProximity)
		{
			continue;
		}
		Contact.FeatureId = Feature(Contact.Normal, Hit.FaceIndex) ^ (static_cast<uint32>(Hit.Item + 1) * 0x9e3779b9u);
		if (!Q.bProximity && Contact.PenetrationDepth <= Q.SpatialTolerance &&
			FVector::DotProduct(Q.End - Q.Start, Contact.Normal) >= -1.e-9)
		{
			continue;
		}
		if (!Output.Add(Contact, Q))
		{
			return EBallQueryResult::CapacityExceeded;
		}
	}
	Output.Sort();
	return Output.Count ? EBallQueryResult::Contact : EBallQueryResult::Clear;
}
