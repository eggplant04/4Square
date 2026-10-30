#pragma once

#include "CoreMinimal.h"
#include "Engine/EngineTypes.h"
#include "BallPhysicsConfig.generated.h"

USTRUCT(BlueprintType)
struct FOURSQUAREVR_API FBallSurfaceMaterial
{
	GENERATED_BODY()
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball")
	bool bOverride = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(ClampMin="0", ClampMax="1"))
	double Restitution = 0.8;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(ClampMin="0"))
	double Friction = 0.35;
	bool IsValid() const { return FMath::IsFinite(Restitution) && Restitution >= 0 && Restitution <= 1 && FMath::IsFinite(Friction) && Friction >= 0; }
};

/** Tunable game values, not experimentally calibrated rubber-ball properties. Units are cm, s, kg. */
USTRUCT(BlueprintType)
struct FOURSQUAREVR_API FBallPhysicsConfig
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(ClampMin="1"))
	int32 Version = 1;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(Units="cm", ClampMin="0.01"))
	double Radius = 3.0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(Units="kg", ClampMin="0.001"))
	double Mass = 0.06;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball")
	FVector Gravity = FVector(0, 0, -980.665);
	/** Exponential damping coefficients, s^-1. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(ClampMin="0"))
	double LinearDrag = 0.02;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(ClampMin="0"))
	double AngularDrag = 0.1;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(ClampMin="0", ClampMax="1"))
	double Restitution = 0.8;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(ClampMin="0"))
	double SurfaceFriction = 0.35;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(ClampMin="0"))
	double SpinFriction = 0.1;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(ClampMin="0"))
	double RollingResistance = 0.01;
	/** cm/s and rad/s; zero disables the designer cap. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(ClampMin="0"))
	double MaximumLinearSpeed = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(ClampMin="0"))
	double MaximumAngularSpeed = 0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(ClampMin="0"))
	double SleepSpeed = 1.0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(ClampMin="0"))
	double WakeSpeed = 2.0;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(ClampMin="0"))
	double SleepAngularSpeed = 0.1;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(ClampMin="0"))
	double SleepDwellSeconds = 0.25;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(ClampMin="0"))
	double MinimumImpactSpeed = 20.0; // cm/s, restitution suppression for small contact chatter.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(Units="s", ClampMin="0.000000001"))
	double TOITolerance = 1.e-7;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(ClampMin="1", ClampMax="64"))
	int32 ManifoldIterations = 12;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball|Collision")
	TEnumAsByte<ECollisionChannel> QueryChannel = ECC_WorldDynamic;
	/** Optional component profile filter, not a replacement global collision profile. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball|Collision")
	FName RequiredCollisionProfile = NAME_None;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball|Collision")
	bool bTraceComplex = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball|Collision")
	bool bUsePhysicalMaterials = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball|Collision", meta=(ClampMin="1", ClampMax="4096"))
	int32 MaximumWorldCandidates = 64;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(Units="cm", ClampMin="0.000001"))
	double CCDTolerance = 0.01;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(Units="cm", ClampMin="0.000001"))
	double PenetrationTolerance = 0.02;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(ClampMin="1", ClampMax="64"))
	int32 MaximumCollisionIterations = 12;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(ClampMin="1", ClampMax="64"))
	int32 MaximumMicrointervals = 8;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball", meta=(ClampMin="1", ClampMax="1000"))
	double FixedFrequency = 120;

	bool IsValid() const;
	double GetFixedDeltaTime() const { return 1.0 / FixedFrequency; }
	double GetSolidSphereInertia() const { return 0.4 * Mass * Radius * Radius; }
};

/** Capacities must cover the configured window. Editing live settings will require a timeline boundary. */
USTRUCT(BlueprintType)
struct FOURSQUAREVR_API FBallNetworkSettings
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball|History", meta=(Units="s", ClampMin="0"))
	double RevisionWindowSeconds = 0.25;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball|History", meta=(ClampMin="2", ClampMax="4096"))
	int32 HistoryCapacity = 256;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball|History", meta=(ClampMin="1", ClampMax="4095"))
	int32 MaximumReplayFrames = 32;

	bool IsValid(double FixedFrequency) const;
	int32 GetRevisionFrames(double FixedFrequency) const;
};

/** Eligibility only: future event code must ALSO block commitment on admitted pending replay/pauses. */
struct FOURSQUAREVR_API FBallFinalityPolicy
{
	static bool CalculateEligibleBoundary(uint64 HeadBoundary, const FBallNetworkSettings& Settings,
		double FixedFrequency, uint64& OutBoundary);
	static bool IsSequenceNewer(uint32 Candidate, uint32 Reference);
};
