#include "Ball/BallHistoryBuffer.h"

bool FBallHistoryFrame::IsValid() const
{
	return State.IsValid() && Inputs.IsValid() && State.SimulationFrame == Inputs.SimulationFrame
		&& State.ConfigVersion == Inputs.ConfigVersion && State.MaterialVersion == Inputs.MaterialVersion
		&& State.EnvironmentRevision == Inputs.EnvironmentRevision;
}

bool FBallHistoryBuffer::Initialize(int32 InCapacity, const FBallIdentity& Identity)
{
	if (InCapacity < 2 || InCapacity > 4096 || !Identity.IsValid()) { return false; }
	Slots.SetNum(InCapacity);
	return Reset(Identity);
}

bool FBallHistoryBuffer::Reset(const FBallIdentity& Identity)
{
	if (!Identity.IsValid() || Slots.Num() < 2) { return false; }
	ActiveIdentity = Identity;
	Count = 0;
	Oldest = Newest = 0;
	bPinned = false;
	RejectedWrites = 0;
	return true;
}

EBallHistoryWrite FBallHistoryBuffer::Push(const FBallHistoryFrame& Frame)
{
	const auto Reject = [&](EBallHistoryWrite Reason) { ++RejectedWrites; return Reason; };
	if (Slots.Num() < 2 || !Frame.IsValid()) { return Reject(EBallHistoryWrite::InvalidFrame); }
	if (Frame.State.Identity != ActiveIdentity) { return Reject(EBallHistoryWrite::WrongIdentity); }
	const uint64 Number = Frame.State.SimulationFrame;
	if (Count > 0 && Number != Newest + 1) { return Reject(EBallHistoryWrite::NonContiguous); }
	if (Count == Slots.Num() && bPinned && Oldest >= PinnedFrom) { return Reject(EBallHistoryWrite::PinnedCapacity); }
	Slots[Number % Slots.Num()] = Frame;
	if (Count == 0) { Oldest = Number; }
	else if (Count == Slots.Num()) { ++Oldest; }
	if (Count < Slots.Num()) { ++Count; }
	Newest = Number;
	return EBallHistoryWrite::Stored;
}

bool FBallHistoryBuffer::CopyFrame(uint64 Frame, const FBallIdentity& Identity, FBallHistoryFrame& OutFrame) const
{
	if (Count == 0 || Identity != ActiveIdentity || Frame < Oldest || Frame > Newest) { return false; }
	const FBallHistoryFrame& Stored = Slots[Frame % Slots.Num()];
	if (Stored.State.SimulationFrame != Frame || Stored.State.Identity != Identity) { return false; }
	OutFrame = Stored;
	return true;
}

bool FBallHistoryBuffer::PinFromFrame(uint64 Frame)
{
	if (Count == 0 || Frame < Oldest || Frame > Newest) { return false; }
	PinnedFrom = Frame;
	bPinned = true;
	return true;
}
