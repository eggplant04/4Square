#pragma once
#include "Ball/BallHandTypes.h"
enum class EBallMotionShape : uint8
{
	Sphere,
	TranslatingBox,
	RoundedBox
};
struct FOURSQUAREVR_API FBallMotionSegment
{
	/** World palm quaternion endpoints. Shortest arc; normalized; immutable replay inputs. */
	FQuat StartOrientation = FQuat::Identity, EndOrientation = FQuat::Identity;
	// Keep 16-byte-aligned quaternions together: same values/precision, fewer padding bytes per retained segment.
	FVector Start = FVector::ZeroVector, End = FVector::ZeroVector, SurfaceAngularVelocity = FVector::ZeroVector;
	double StartFraction = 0, EndFraction = 1;
	uint32 FirstSample = 0, SecondSample = 0;
};
struct FOURSQUAREVR_API FBallColliderMotion
{
	FBallColliderId Id;
	EBallContactCategory Category = EBallContactCategory::Hand;
	EBallMotionShape Shape = EBallMotionShape::Sphere;
	uint32 Participant = 0, OriginRevision = 0;
	uint32 ConfigurationVersion = 1;
	double WorldToMeters = 100;
	EBallHandSide Side = EBallHandSide::Left;
	double Radius = 7;
	FVector HalfExtents = FVector(5);
	FQuat Orientation = FQuat::Identity;
	double RoundRadius = 0;
	double CCDDistanceTolerance = 1.e-6;
	int32 MaximumCCDIterations = 256;
	/** Controller-to-center offset in PALM coordinates. RoundedBox reconstructs the exact offset arc. */
	FVector PivotToCenter = FVector::ZeroVector;
	FQuat CalibrationRotation = FQuat::Identity;
	FBallPalmFaceMaterials FaceMaterials;
	FBallSurfaceMaterial Material;
	int32 FirstSegment = 0, SegmentCount = 0;
};
/** Immutable numeric input for a complete fixed step, shared by all balls and replay. */
struct FOURSQUAREVR_API FBallMotionFrame
{
	uint64 Frame = 0;
	FGuid Epoch;
	uint32 Timebase = 1, Revision = 1, EnvironmentRevision = 1;
	uint32 GeometryVersion = BallMotionLimits::GeometryVersion;
	double DeltaTime = 1.0 / 120.0;
	int32 ColliderCount = 0, SegmentCount = 0;
	FBallColliderMotion Colliders[BallMotionLimits::Colliders];
	FBallMotionSegment Segments[BallMotionLimits::Segments];
	bool IsValid() const;
	bool Add(const FBallColliderMotion &Collider, TConstArrayView<FBallMotionSegment> Motion);
};
/** Fixed allocation; failed writes and missing history never silently use live poses. */
class FOURSQUAREVR_API FBallMotionHistory
{
  public:
	bool Initialize(int32 Capacity);
	bool CanPush(const FBallMotionFrame &Frame) const;
	bool Push(const FBallMotionFrame &Frame);
	const FBallMotionFrame *Find(uint64 Frame, uint32 Timebase, uint32 Revision) const;
	void Reset();
	void PinFrom(uint64 Frame)
	{
		PinnedFrom = Frame;
	}
	void Unpin()
	{
		PinnedFrom = MAX_uint64;
	}
	SIZE_T GetAllocatedBytes() const
	{
		return Frames.GetAllocatedSize();
	}
	int32 GetCapacity() const
	{
		return Frames.Num();
	}
	int32 GetRetainedCount() const
	{
		return Count;
	}
	SIZE_T GetActivePayloadBytes() const;

  private:
	TArray<FBallMotionFrame> Frames;
	int32 Head = 0, Count = 0;
	uint64 PinnedFrom = MAX_uint64;
};
