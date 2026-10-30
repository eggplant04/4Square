#pragma once

#include "CoreMinimal.h"
#include "Ball/BallPhysicsConfig.h"

namespace BallLimits
{
inline constexpr int32 Contacts = 8;
inline constexpr int32 ForceProviders = 4;
inline constexpr int32 StepEvents = 64;
inline constexpr int32 MotionPairs = 12;
} // namespace BallLimits

struct FOURSQUAREVR_API FBallIdentity
{
	FGuid Epoch;
	uint64 BallId = 0;
	uint32 Generation = 0;
	bool IsValid() const
	{
		return Epoch.IsValid() && BallId != 0 && Generation != 0;
	}
	bool operator==(const FBallIdentity &Other) const
	{
		return Epoch == Other.Epoch && BallId == Other.BallId && Generation == Other.Generation;
	}
	bool operator!=(const FBallIdentity &Other) const
	{
		return !(*this == Other);
	}
};

struct FOURSQUAREVR_API FBallColliderId
{
	uint64 Value = 0;
	uint32 Generation = 0;
	bool IsValid() const
	{
		return Value != 0 && Generation != 0;
	}
	bool operator==(const FBallColliderId &Other) const
	{
		return Value == Other.Value && Generation == Other.Generation;
	}
};

struct FOURSQUAREVR_API FBallFrameTime
{
	uint64 Frame = 0;
	double Fraction = 0;
	bool IsValid() const
	{
		return Frame != MAX_uint64 && FMath::IsFinite(Fraction) && Fraction >= 0 && Fraction < 1;
	}
};

enum class EBallActivityState : uint8
{
	Active,
	Resting
};
enum class EBallContactCategory : uint8
{
	Court,
	Wall,
	Hand,
	Kinematic,
	Environment
};
enum class EBallStepResult : uint8
{
	Success,
	InvalidConfiguration,
	InvalidState,
	InvalidInputs,
	IncompatibleEnvironment,
	SubdivisionLimit,
	QueryFailure,
	ContactSolverRequired,
	NumericFailure,
	CollisionLimit,
	ContactCapacity,
	PenetrationFailure
};

/** Persistent contact episode/manifold data; it is part of checkpoints, not cosmetic event history. */
struct FOURSQUAREVR_API FBallContactState
{
	FBallColliderId Collider;
	uint32 FeatureId = 0;
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

/** Persistent dynamic contact episode; cache eviction must not re-arm a palm hit. */
enum class EBallMotionBlockReason : uint8
{
	None,
	InitialOverlap,
	ConflictingContact
};
struct FOURSQUAREVR_API FBallMotionPairState
{
	FBallColliderId Collider;
	uint32 Episode = 0, EventCounterAfterImpact = 0;
	bool bArmed = false, bBlocked = false;
	EBallMotionBlockReason BlockReason = EBallMotionBlockReason::None;
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
	int32 MotionPairCount = 0;
	FBallMotionPairState MotionPairs[BallLimits::MotionPairs];

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
	uint32 MotionTimebase = 0, MotionRevision = 0; // Both zero means static-only, never live-pose fallback.
	int32 ForceCount = 0;
	FBallForceInput Forces[BallLimits::ForceProviders];
	bool IsValid() const;
};

struct FOURSQUAREVR_API FBallContact
{
	FBallColliderId Collider;
	uint32 FeatureId = 0;
	FBallSurfaceMaterial Material;
	EBallContactCategory Category = EBallContactCategory::Environment;
	double TimeOfImpact = 0; // Normalized within this query's drift interval.
	FVector Position = FVector::ZeroVector;
	FVector Normal = FVector::UpVector; // Obstacle -> ball.
	FVector SurfaceVelocity = FVector::ZeroVector;
	double PenetrationDepth = 0;
	double Separation = 0;
	bool bAllowImpact = true;
	uint32 Participant = 0, OriginRevision = 0, FirstSample = 0, SecondSample = 0;
	uint8 HandSide = 0;
};

struct FOURSQUAREVR_API FBallCollisionEvent
{
	FBallIdentity Ball;
	EBallContactCategory Category = EBallContactCategory::Environment;
	FBallFrameTime Time;
	FBallColliderId Collider;
	uint32 FeatureId = 0;
	uint32 Episode = 0;
	uint32 Ordinal = 0;
	FVector Position = FVector::ZeroVector;
	FVector Normal = FVector::UpVector;
	FVector IncomingVelocity = FVector::ZeroVector;
	FVector OutgoingVelocity = FVector::ZeroVector;
	FVector IncomingAngularVelocity = FVector::ZeroVector;
	FVector OutgoingAngularVelocity = FVector::ZeroVector;
	double NormalImpulse = 0;
	uint32 Participant = 0, OriginRevision = 0, FirstSample = 0, SecondSample = 0;
	uint8 HandSide = 0;
};

struct FOURSQUAREVR_API FBallStepDiagnostics
{
	uint32 Microintervals = 0;
	uint32 Queries = 0;
	uint32 CollisionIterations = 0;
	uint32 PenetrationCorrections = 0;
	uint32 RestitutionFallbacks = 0;
	uint32 SupportForceSolves = 0;
	uint32 MotionSlices = 0, RejectedMotionContacts = 0;
	double DurationMicroseconds = 0;
};

struct FOURSQUAREVR_API FBallStepOutput
{
	EBallStepResult Result = EBallStepResult::InvalidInputs;
	FBallStepDiagnostics Diagnostics;
	int32 ContactCount = 0;
	FBallContact Contacts[BallLimits::StepEvents];
	int32 EventCount = 0;
	FBallCollisionEvent Events[BallLimits::StepEvents];
};
