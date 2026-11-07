#pragma once
#include "CoreMinimal.h"
#include "Ball/BallHandTypes.h"
#include "BallNetworkTypes.generated.h"

namespace BallNet
{
inline constexpr uint16 Protocol = 1;
inline constexpr int32 MaximumPacketBytes = 8192, MaximumPoseSamples = 12, PoseHistory = 256;
} // namespace BallNet

/** Bounded envelope. Counts are checked before allocation, including during Unreal RPC/property decoding. */
USTRUCT()
struct FOURSQUAREVR_API FBallWirePacket
{
	GENERATED_BODY()
	UPROPERTY() TArray<uint8> Bytes;
	bool NetSerialize(FArchive &Ar, UPackageMap *Map, bool &Success);
	bool operator==(const FBallWirePacket &B) const
	{
		return Bytes == B.Bytes;
	}
};
template <> struct TStructOpsTypeTraits<FBallWirePacket> : TStructOpsTypeTraitsBase2<FBallWirePacket>
{
	enum
	{
		WithNetSerializer = true,
		WithIdenticalViaEquality = true
	};
};

USTRUCT(BlueprintType)
struct FOURSQUAREVR_API FBallTransportSettings
{
	GENERATED_BODY()
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Network") double PoseRate = 60;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Network") double MinimumPoseRate = 30;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Network") double SnapshotRate = 25;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Network") double RemotePoseRate = 30;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Network") double MaximumRPCsPerSecond = 90;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Network") double MaximumSamplesPerSecond = 240;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Network") double MaximumPoseAge = .5;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Network") double MaximumFuturePose = .025;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Network") double MaximumClockUncertainty = .1;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Network") double MaximumClockDriftPPM = 500;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Network") double OriginPositionTolerance = .02;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Network") double OriginAngularTolerance = .001;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Network") double VisualDelay = .04;
	bool IsValid() const;
};

/** No identity/dimensions supplied by the client can override this server-owned profile. */
struct FOURSQUAREVR_API FBallPeerProfile
{
	FGuid Epoch;
	uint32 Participant = 0, Version = 1, Timebase = 1, OriginRevision = 1;
	double EffectiveTimelineSeconds = 0;
	uint64 EffectiveFrame = 0;
	FBallPalmSettings Left, Right;
	bool bEnableLeft = true, bEnableRight = true;
	FBallHandTrackingSettings Tracking;
	FBallTransportSettings Transport;
	FVector Origin = FVector::ZeroVector;
	FQuat OriginRotation = FQuat::Identity;
	double WorldToMeters = 100;
	bool IsValid() const;
};
struct FOURSQUAREVR_API FBallPoseBatch
{
	FGuid Epoch;
	uint32 Timebase = 1, ProfileVersion = 1, OriginRevision = 1, Sequence = 1, CaptureGeneration = 1;
	double SentSeconds = 0;
	int32 Count = 0;
	FBallHandObservation Samples[BallNet::MaximumPoseSamples];
	uint32 Generations[BallNet::MaximumPoseSamples] = {}; // Independent left/right tracking lifetimes.
};
struct FOURSQUAREVR_API FBallNetworkSnapshot
{
	FBallState State;
	FBallPhysicsConfig Physics;
	FBallNetworkSettings History;
	uint32 Sequence = 1, Timebase = 1, TimelineRevision = 1, GeometryVersion = BallMotionLimits::GeometryVersion;
	double ServerSeconds = 0, TimelineSeconds = 0;
	bool bPaused = false;
	bool IsValid() const;
};
/** Explicit field codec. Pose quantization is 0.001 cm and smallest-three 16-bit quaternion components. */
struct FOURSQUAREVR_API FBallNetworkCodec
{
	static bool WriteProfile(const FBallPeerProfile &Profile, FBallWirePacket &Out);
	static bool ReadProfile(const FBallWirePacket &Packet, FBallPeerProfile &Out);
	static bool WritePoses(const FBallPoseBatch &Batch, FBallWirePacket &Out, bool Remote = false);
	static bool ReadPoses(const FBallWirePacket &Packet, FBallPoseBatch &Out, bool Remote = false);
	static bool WriteSnapshot(const FBallNetworkSnapshot &Snapshot, FBallWirePacket &Out);
	static bool ReadSnapshot(const FBallWirePacket &Packet, FBallNetworkSnapshot &Out);
};
enum class EBallInputDisposition : uint8
{
	Accepted,
	Duplicate,
	Malformed,
	WrongOwner,
	WrongContext,
	ClockUncertain,
	TooOld,
	Future,
	OutOfOrder,
	ConflictingDuplicate,
	InvalidOrigin,
	InvalidMotion,
	RateLimited,
	Capacity,
	LateHistorical
};
struct FOURSQUAREVR_API FBallNetworkStatistics
{
	uint64 Packets = 0, Bytes = 0, Accepted = 0, Duplicates = 0, Rejected = 0, Late = 0, SequenceGaps = 0;
	double DecodeMicroseconds = 0, AdmissionMicroseconds = 0;
};
/** One canonical, self-contained authoritative view; no prediction or visual smoothing. */
struct FOURSQUAREVR_API FBallAuthoritativeView
{
	bool bReady = false;
	FBallNetworkSnapshot Latest;
	bool Accept(const FBallNetworkSnapshot &Snapshot);
};
