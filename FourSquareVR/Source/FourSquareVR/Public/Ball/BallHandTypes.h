#pragma once
#include "CoreMinimal.h"
#include "Ball/BallPhysicsTypes.h"
#include "BallHandTypes.generated.h"

UENUM(BlueprintType)
enum class EBallHandSide : uint8
{
	Left,
	Right
};
UENUM(BlueprintType)
enum class EBallTrackingRejection : uint8
{
	None,
	WarmingUp,
	Untracked,
	InvalidPose,
	OutOfOrder,
	Stale,
	OriginChanged,
	Discontinuity,
	ExcessiveVelocity,
	ExcessiveAcceleration,
	ExcessiveReach,
	Capacity,
	UnsupportedRotation,
	ConflictingContact
};
USTRUCT(BlueprintType)
struct FOURSQUAREVR_API FBallPalmSettings
{
	GENERATED_BODY()
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Palm", meta = (ClampMin = "0.01", Units = "cm"))
	double Radius = 7;
	/** Controller-local physical palm center, in cm. Calibrate per controller/device. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Palm")
	FVector CenterOffset = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Palm")
	FBallSurfaceMaterial Material;
	/** Adds tangential sphere-surface rotation; offset-center motion is already included once. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Palm")
	bool bRotatingSurfaceFriction = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Palm", meta = (ClampMin = "0.0001"))
	double MaximumCurveError = 0.02;
	bool IsValid() const;
	bool operator==(const FBallPalmSettings &B) const
	{
		return Radius == B.Radius && CenterOffset == B.CenterOffset && Material.Restitution == B.Material.Restitution &&
			   Material.Friction == B.Material.Friction && Material.bOverride == B.Material.bOverride &&
			   bRotatingSurfaceFriction == B.bRotatingSurfaceFriction && MaximumCurveError == B.MaximumCurveError;
	}
};
USTRUCT(BlueprintType)
struct FOURSQUAREVR_API FBallHandTrackingSettings
{
	GENERATED_BODY()
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Tracking", meta = (ClampMin = "0.0001"))
	double MaximumSampleGap = 0.05;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Tracking", meta = (ClampMin = "0"))
	double MaximumExtrapolation = 1.0 / 60.0;
	/** Filters only unknown future translation; observed trajectory/impact velocity remain consistent. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Tracking", meta = (ClampMin = "0"))
	double ExtrapolationFilterSeconds = .008;
	/** Maximum extra cm of extrapolation error introduced by filtering, relative to raw constant velocity. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Tracking", meta = (ClampMin = "0"))
	double MaximumExtrapolationFilterError = .1;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Tracking", meta = (ClampMin = "1"))
	double MaximumSpeed = 2500;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Tracking", meta = (ClampMin = "1"))
	double MaximumAngularSpeed = 80;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Tracking", meta = (ClampMin = "1"))
	double MaximumAcceleration = 200000;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Tracking", meta = (ClampMin = "1"))
	double MaximumHeadSpeed = 1500;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Tracking", meta = (ClampMin = "1"))
	double MaximumReach = 200;
	bool IsValid() const;
	bool operator==(const FBallHandTrackingSettings &B) const
	{
		return MaximumSampleGap == B.MaximumSampleGap && MaximumExtrapolation == B.MaximumExtrapolation &&
			   ExtrapolationFilterSeconds == B.ExtrapolationFilterSeconds &&
			   MaximumExtrapolationFilterError == B.MaximumExtrapolationFilterError && MaximumSpeed == B.MaximumSpeed &&
			   MaximumAngularSpeed == B.MaximumAngularSpeed && MaximumAcceleration == B.MaximumAcceleration &&
			   MaximumHeadSpeed == B.MaximumHeadSpeed && MaximumReach == B.MaximumReach;
	}
};
/** Numeric observation time, NOT a fabricated hardware sensor timestamp. */
struct FOURSQUAREVR_API FBallHandObservation
{
	double CaptureSeconds = 0, TimelineSeconds = 0;
	uint32 Timebase = 1, OriginRevision = 1, Sequence = 1;
	uint32 ParticipantId = 1;
	uint32 ConfigurationVersion = 1;
	double WorldToMeters = 100;
	EBallHandSide Side = EBallHandSide::Left;
	FVector ControllerPosition = FVector::ZeroVector;
	FQuat ControllerOrientation = FQuat::Identity;
	FVector HeadPosition = FVector::ZeroVector;
	FQuat HeadOrientation = FQuat::Identity;
	FVector OriginPosition = FVector::ZeroVector;
	FQuat OriginOrientation = FQuat::Identity;
	bool bTracked = true;
	bool IsValid() const;
};
struct FOURSQUAREVR_API FBallHandSample
{
	FBallHandObservation Pose;
	uint32 Generation = 1;
	FVector LinearVelocity = FVector::ZeroVector, AngularVelocity = FVector::ZeroVector;
	FVector ExtrapolationVelocity = FVector::ZeroVector;
	EBallTrackingRejection Rejection = EBallTrackingRejection::WarmingUp;
	bool bCollisionReady = false;
};
namespace BallMotionLimits
{
inline constexpr int32 Hands = 8, Kinematics = 4, Colliders = Hands + Kinematics, SegmentsPerCollider = 8,
					   Segments = Colliders * SegmentsPerCollider;
inline constexpr uint64 HandNamespace = 0x8000000000000000ull, KinematicNamespace = 0xc000000000000000ull,
						StaticMask = 0x3fffffffffffffffull;
inline FBallColliderId HandId(uint32 Participant, EBallHandSide Side, uint32 Generation)
{
	return {HandNamespace | (uint64(Participant) << 1) | uint64(Side), Generation};
}
} // namespace BallMotionLimits
