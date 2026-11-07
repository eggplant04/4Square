#pragma once
#include "Ball/BallNetworkTypes.h"
#include "Ball/BallNetworkClock.h"
#include "Ball/BallHandPoseHistory.h"

/** Sorted fixed-capacity accepted measurements, with frozen timestamps and causal derived/filter state. */
class FOURSQUAREVR_API FBallNetworkPoseHistory
{
  public:
	void Initialize()
	{
		Samples.SetNum(BallNet::PoseHistory);
		Count = 0;
		PinnedFrom = TNumericLimits<double>::Max();
	}
	EBallInputDisposition Insert(const FBallHandObservation &Pose, uint32 Generation,
								 const FBallHandTrackingSettings &Settings);
	TConstArrayView<FBallHandSample> View() const
	{
		return MakeArrayView(Samples.GetData(), Count);
	}
	int32 Num() const
	{
		return Count;
	}
	SIZE_T GetAllocatedBytes() const
	{
		return Samples.GetAllocatedSize();
	}
	void PinFrom(double Timeline)
	{
		PinnedFrom = Timeline;
	}
	void Unpin()
	{
		PinnedFrom = TNumericLimits<double>::Max();
	}

  private:
	TArray<FBallHandSample> Samples;
	int32 Count = 0;
	double PinnedFrom = TNumericLimits<double>::Max();
};
class FOURSQUAREVR_API FBallPoseAdmission
{
  public:
	void Initialize(const FBallPeerProfile &Profile);
	void ChangeProfile(const FBallPeerProfile &NewProfile);
	EBallInputDisposition Admit(const FBallPoseBatch &Batch, const FBallClockEstimate &Clock, bool ClockReady,
								double ServerReceiveSeconds, double TimelineNow, double SimulatedThrough = -1);
	bool BuildMotion(double Start, FBallMotionFrame &Frame, EBallTrackingRejection *LeftReason = nullptr,
					 EBallTrackingRejection *RightReason = nullptr) const;
	void PinFrom(double Timeline)
	{
		for (auto &H : Hands)
			H.PinFrom(Timeline);
	}
	void Unpin()
	{
		for (auto &H : Hands)
			H.Unpin();
	}
	const FBallNetworkPoseHistory &Hand(EBallHandSide Side) const
	{
		return Hands[uint8(Side)];
	}
	SIZE_T GetAllocatedBytes() const
	{
		return Hands[0].GetAllocatedBytes() + Hands[1].GetAllocatedBytes();
	}
	void RecordTransport(SIZE_T Bytes, double DecodeUs)
	{
		Statistics.Bytes += Bytes;
		Statistics.DecodeMicroseconds += DecodeUs;
	}
	FBallNetworkStatistics Statistics;
	FBallPeerProfile Profile;
	double EarliestDirtyTimeline = TNumericLimits<double>::Max();

  private:
	FBallNetworkPoseHistory Hands[2];
	uint32 LastPacketSequence = 0, LastCaptureGeneration[2] = {}, MotionGeneration[2] = {1, 1};
	struct FGeneration
	{
		uint32 Client = 0, Server = 0;
	} Generations[2][16];
	int32 GenerationHead[2] = {};
	FBallTokenBucket Rate;
	FBallTokenBucket SampleRates[2];
};
