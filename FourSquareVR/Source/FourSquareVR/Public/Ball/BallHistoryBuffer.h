#pragma once

#include "Ball/BallPhysicsTypes.h"

/** Owned copies, not references into mutable input queues. State(F) and the input for step F. */
struct FOURSQUAREVR_API FBallHistoryFrame
{
	FBallState State;
	FBallStepInputs Inputs;
	bool IsValid() const;
};

enum class EBallHistoryWrite : uint8 { Stored, InvalidFrame, WrongIdentity, NonContiguous, PinnedCapacity };

/** Allocate once at initialization. Never overwrites pinned replay inputs; callers receive copies. */
class FOURSQUAREVR_API FBallHistoryBuffer
{
public:
	bool Initialize(int32 Capacity, const FBallIdentity& Identity);
	bool Reset(const FBallIdentity& Identity);
	EBallHistoryWrite Push(const FBallHistoryFrame& Frame);
	bool CopyFrame(uint64 Frame, const FBallIdentity& Identity, FBallHistoryFrame& OutFrame) const;
	bool PinFromFrame(uint64 Frame);
	void Unpin() { bPinned = false; }
	int32 Num() const { return Count; }
	int32 Capacity() const { return Slots.Num(); }
	uint64 GetOldestFrame() const { return Oldest; }
	uint64 GetNewestFrame() const { return Newest; }
	SIZE_T GetAllocatedBytes() const { return sizeof(*this) + Slots.GetAllocatedSize(); }
	uint64 GetRejectedWrites() const { return RejectedWrites; }

private:
	TArray<FBallHistoryFrame> Slots;
	FBallIdentity ActiveIdentity;
	uint64 Oldest = 0;
	uint64 Newest = 0;
	uint64 PinnedFrom = 0;
	uint64 RejectedWrites = 0;
	int32 Count = 0;
	bool bPinned = false;
};
