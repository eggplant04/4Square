#include "Ball/BallMotionHistory.h"
bool FBallMotionFrame::IsValid() const
{
	if (Frame == MAX_uint64 || !Epoch.IsValid() || !Timebase || !Revision || !EnvironmentRevision ||
		!FMath::IsFinite(DeltaTime) || DeltaTime <= 0 || ColliderCount < 0 ||
		ColliderCount > BallMotionLimits::Colliders || SegmentCount < 0 || SegmentCount > BallMotionLimits::Segments)
	{
		return false;
	}
	int32 Expected = 0;
	for (int32 I = 0; I < ColliderCount; ++I)
	{
		const auto &C = Colliders[I];
		if (!C.Id.IsValid() || !C.ConfigurationVersion || !FMath::IsFinite(C.WorldToMeters) || C.WorldToMeters <= 0 ||
			C.FirstSegment != Expected || C.SegmentCount < 1 ||
			C.SegmentCount > BallMotionLimits::SegmentsPerCollider || C.FirstSegment + C.SegmentCount > SegmentCount ||
			!C.Material.IsValid() || !FMath::IsFinite(C.Radius) || C.Radius <= 0 || C.HalfExtents.ContainsNaN() ||
			C.HalfExtents.GetMin() <= 0 || C.Orientation.ContainsNaN() || !C.Orientation.IsNormalized() ||
			uint8(C.Shape) > uint8(EBallMotionShape::TranslatingBox) ||
			(C.Category != EBallContactCategory::Hand && C.Category != EBallContactCategory::Kinematic))
		{
			return false;
		}
		for (int32 J = 0; J < I; ++J)
		{
			if (Colliders[J].Id == C.Id)
			{
				return false;
			}
		}
		double Last = 0;
		for (int32 J = 0; J < C.SegmentCount; ++J)
		{
			const auto &S = Segments[Expected + J];
			if (!FMath::IsFinite(S.StartFraction) || !FMath::IsFinite(S.EndFraction) ||
				S.StartFraction < Last - 1.e-12 || S.StartFraction < 0 || S.EndFraction > 1 ||
				S.EndFraction <= S.StartFraction || S.Start.ContainsNaN() || S.End.ContainsNaN() ||
				S.SurfaceAngularVelocity.ContainsNaN())
			{
				return false;
			}
			Last = S.EndFraction;
		}
		Expected += C.SegmentCount;
	}
	return Expected == SegmentCount;
}
bool FBallMotionFrame::Add(const FBallColliderMotion &Collider, TConstArrayView<FBallMotionSegment> Motion)
{
	if (ColliderCount == BallMotionLimits::Colliders || Motion.Num() < 1 ||
		Motion.Num() > BallMotionLimits::SegmentsPerCollider ||
		SegmentCount + Motion.Num() > BallMotionLimits::Segments)
	{
		return false;
	}
	auto C = Collider;
	C.FirstSegment = SegmentCount;
	C.SegmentCount = Motion.Num();
	Colliders[ColliderCount++] = C;
	for (const auto &S : Motion)
	{
		Segments[SegmentCount++] = S;
	}
	if (!IsValid())
	{
		--ColliderCount;
		SegmentCount -= Motion.Num();
		return false;
	}
	return true;
}
bool FBallMotionHistory::Initialize(int32 Capacity)
{
	if (Capacity < 2 || Capacity > 4096)
	{
		return false;
	}
	Frames.SetNum(Capacity);
	Reset();
	return true;
}
void FBallMotionHistory::Reset()
{
	Head = 0;
	Count = 0;
	PinnedFrom = MAX_uint64;
}
bool FBallMotionHistory::CanPush(const FBallMotionFrame &F) const
{
	if (!F.IsValid() || Frames.Num() < 2)
	{
		return false;
	}
	if (Count)
	{
		const auto &Last = Frames[(Head + Count - 1) % Frames.Num()];
		if (F.Epoch != Last.Epoch || F.Timebase != Last.Timebase || F.Revision != Last.Revision ||
			F.Frame != Last.Frame + 1 || F.EnvironmentRevision != Last.EnvironmentRevision)
		{
			return false;
		}
	}
	return Count < Frames.Num() || PinnedFrom == MAX_uint64 || Frames[Head].Frame < PinnedFrom;
}
bool FBallMotionHistory::Push(const FBallMotionFrame &F)
{
	if (!CanPush(F))
	{
		return false;
	}
	if (Count == Frames.Num())
	{
		Head = (Head + 1) % Frames.Num();
		--Count;
	}
	Frames[(Head + Count) % Frames.Num()] = F;
	++Count;
	return true;
}
const FBallMotionFrame *FBallMotionHistory::Find(uint64 Frame, uint32 Timebase, uint32 Revision) const
{
	if (!Count || Frame < Frames[Head].Frame || Frame - Frames[Head].Frame >= uint64(Count))
	{
		return nullptr;
	}
	const auto &F = Frames[(Head + int32(Frame - Frames[Head].Frame)) % Frames.Num()];
	return F.Frame == Frame && F.Timebase == Timebase && F.Revision == Revision ? &F : nullptr;
}
