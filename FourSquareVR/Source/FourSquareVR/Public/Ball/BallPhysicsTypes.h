#pragma once

#include "CoreMinimal.h"

namespace BallLimits
{
	inline constexpr int32 Contacts = 8;
	inline constexpr int32 ForceProviders = 4;
}

struct FOURSQUAREVR_API FBallIdentity
{
	FGuid Epoch;
	uint64 BallId = 0;
	uint32 Generation = 0;
	bool IsValid() const { return Epoch.IsValid() && BallId != 0 && Generation != 0; }
	bool operator==(const FBallIdentity& Other) const
	{ return Epoch == Other.Epoch && BallId == Other.BallId && Generation == Other.Generation; }
	bool operator!=(const FBallIdentity& Other) const { return !(*this == Other); }
};

struct FOURSQUAREVR_API FBallColliderId
{
	uint64 Value = 0;
	uint32 Generation = 0;
	bool IsValid() const { return Value != 0 && Generation != 0; }
	bool operator==(const FBallColliderId& Other) const { return Value == Other.Value && Generation == Other.Generation; }
};

struct FOURSQUAREVR_API FBallFrameTime
{
	uint64 Frame = 0;
	double Fraction = 0;
	bool IsValid() const { return Frame != MAX_uint64 && FMath::IsFinite(Fraction) && Fraction >= 0 && Fraction < 1; }
};

enum class EBallActivityState : uint8 { Active, Resting };
enum class EBallContactCategory : uint8 { Court, Wall, Hand, Kinematic };
enum class EBallStepResult : uint8
{
	Success, InvalidConfiguration, InvalidState, InvalidInputs, IncompatibleEnvironment,
	SubdivisionLimit, QueryFailure, ContactSolverRequired, NumericFailure
};

/** Persistent contact episode/manifold data; it is part of checkpoints, not cosmetic event history. */
struct FOURSQUAREVR_API FBallContactState
{
	FBallColliderId Collider;
	uint32 Episode = 0;
	uint64 LastImpactFrame = 0;
	FVector Normal = FVector::UpVector;
	FVector ContactPoint = FVector::ZeroVector;
	FVector AccumulatedTangentImpulse = FVector::ZeroVector;
	double AccumulatedNormalImpulse = 0;
	double Separation = 0;
	bool bTouching = false;
	bool bSupporting = false;
	bool bRestitutionApplied = false;
	bool IsValid() const;
};

struct FOURSQUAREVR_API FBallForceState
{
	uint32 ProviderId = 0;
	uint32 Version = 0;
	uint64 Counter = 0;
	double Values[4] = {};
	bool IsValid() const;
};

/** Complete dynamic boundary state. No UObject references or variable-sized storage. */
struct FOURSQUAREVR_API FBallState
{
	FBallIdentity Identity;
	uint64 SimulationFrame = 0; // State(F) is the START of fixed frame F.
	uint32 ConfigVersion = 1;
	uint32 MaterialVersion = 1;
	uint32 EnvironmentRevision = 1;
	FVector Position = FVector::ZeroVector;
	FVector LinearVelocity = FVector::ZeroVector;
	FQuat Rotation = FQuat::Identity;
	FVector AngularVelocity = FVector::ZeroVector;
	EBallActivityState Activity = EBallActivityState::Active;
	double SleepDwellTime = 0;
	bool bHasSupport = false;
	FBallColliderId SupportCollider;
	FVector SupportNormal = FVector::UpVector;
	FVector SupportVelocity = FVector::ZeroVector;
	uint64 DeterministicSeed = 0;
	uint32 NextEventOrdinal = 0;
	int32 ContactCount = 0;
	FBallContactState Contacts[BallLimits::Contacts];
	int32 ForceStateCount = 0;
	FBallForceState ForceStates[BallLimits::ForceProviders];

	bool IsValid() const;
};

/** Already sampled, explicitly ordered accelerations; no hidden reads of Actors, time, or randomness. */
struct FOURSQUAREVR_API FBallForceInput
{
	uint32 ProviderId = 0;
	int32 Priority = 0;
	FVector LinearAcceleration = FVector::ZeroVector;
	FVector AngularAcceleration = FVector::ZeroVector;
};

struct FOURSQUAREVR_API FBallStepInputs
{
	uint64 SimulationFrame = 0;
	uint32 ConfigVersion = 1;
	uint32 MaterialVersion = 1;
	uint32 EnvironmentRevision = 1;
	int32 ForceCount = 0;
	FBallForceInput Forces[BallLimits::ForceProviders];
	bool IsValid() const;
};

struct FOURSQUAREVR_API FBallContact
{
	FBallColliderId Collider;
	EBallContactCategory Category = EBallContactCategory::Court;
	double TimeOfImpact = 0; // Normalized within this query's drift interval.
	FVector Position = FVector::ZeroVector;
	FVector Normal = FVector::UpVector; // Obstacle -> ball.
	FVector SurfaceVelocity = FVector::ZeroVector;
	double PenetrationDepth = 0;
};

struct FOURSQUAREVR_API FBallStepDiagnostics
{
	uint32 Microintervals = 0;
	uint32 Queries = 0;
	double DurationMicroseconds = 0;
};

struct FOURSQUAREVR_API FBallStepOutput
{
	EBallStepResult Result = EBallStepResult::InvalidInputs;
	FBallStepDiagnostics Diagnostics;
	int32 ContactCount = 0;
	FBallContact Contacts[BallLimits::Contacts];
};
