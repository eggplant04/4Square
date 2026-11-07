#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Ball/BallHandPoseHistory.h"
#include "Ball/BallPoseAdmission.h"
#include "VRHandCollisionComponent.generated.h"

class UMotionControllerComponent;
class UCameraComponent;
class UBallSimulationSubsystem;

/** Samples existing Grips. Network worlds use owned pose RPCs and server-validated numeric history. */
UCLASS(ClassGroup = (Ball), meta = (BlueprintSpawnableComponent))
class FOURSQUAREVR_API UVRHandCollisionComponent : public UActorComponent
{
	GENERATED_BODY()
  public:
	UVRHandCollisionComponent();
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void TickComponent(float Delta, ELevelTick Type, FActorComponentTickFunction *Tick) override;
	virtual void ApplyWorldOffset(const FVector &Offset, bool bWorldShift) override;
	virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty> &Out) const override;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Network") FBallTransportSettings Transport;
	/** Server native navigation validation. Disabling rejects client requests but preserves explicit server approval.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Network") bool bAllowValidatedOriginRequests = true;
	UFUNCTION(BlueprintCallable, BlueprintAuthorityOnly, Category = "Ball|Network")
	bool SetServerPalmConfiguration(const FBallPalmSettings &Left, const FBallPalmSettings &Right);
	UFUNCTION(BlueprintCallable, BlueprintAuthorityOnly, Category = "Ball|Network")
	bool AuthorizeTrackingOrigin(FVector Location, FRotator Rotation);
	void RequestAuthoritativeBaseline();
	double GetEstimatedServerSeconds(double LocalNow) const;
	FString GetNetworkDiagnostics() const;
	const FBallPoseAdmission *GetAdmission() const
	{
		return Admission.Get();
	}
	const FBallPeerProfile &GetAuthorityProfile() const
	{
		return AuthorityProfile;
	}
	const FBallPeerProfile *FindHistoricalProfile(uint32 Version) const;
	const FBallClockEstimate &GetClockEstimate() const
	{
		return ReceivedClock;
	}
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Hands")
	FBallPalmSettings LeftPalm;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Hands")
	FBallPalmSettings RightPalm;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Hands")
	FBallHandTrackingSettings Tracking;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Hands")
	bool bEnableLeft = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Hands")
	bool bEnableRight = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Hands")
	bool bRequireTrackedHead = true;
	/** Explicit opt-in for BallHandTestRig. Never pretends a real device is tracked. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Test")
	bool bAllowScriptedTrackingForTests = false;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Debug")
	bool bDebugDraw = false;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ball|Hands")
	TObjectPtr<UMotionControllerComponent> LeftGrip;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ball|Hands")
	TObjectPtr<UMotionControllerComponent> RightGrip;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ball|Debug")
	EBallTrackingRejection LeftRejection = EBallTrackingRejection::WarmingUp;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ball|Debug")
	EBallTrackingRejection RightRejection = EBallTrackingRejection::WarmingUp;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ball|Debug")
	FVector LeftPalmVelocity = FVector::ZeroVector;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ball|Debug")
	FVector RightPalmVelocity = FVector::ZeroVector;
	UFUNCTION(BlueprintCallable, Category = "Ball|Hands")
	bool RegisterMotionControllers(UMotionControllerComponent *Left, UMotionControllerComponent *Right,
								   UCameraComponent *Head, USceneComponent *TrackingOrigin);
	UFUNCTION(BlueprintCallable, Category = "Ball|Hands")
	bool DiscoverGripControllers();
	UFUNCTION(BlueprintCallable, Category = "Ball|Hands")
	void NotifyTrackingDiscontinuity();
	UFUNCTION(BlueprintPure, Category = "Ball|Hands")
	bool AreControllersRegistered() const;
	uint32 GetParticipantId() const
	{
		return ParticipantId;
	}
	void CaptureTracking();
	void BuildRecordedMotion(FBallMotionFrame &Frame, double StartSeconds);
	SIZE_T GetHistoryAllocatedBytes() const
	{
		return LeftHistory.GetAllocatedBytes() + RightHistory.GetAllocatedBytes();
	}
	const FBallHandPoseHistory &GetHandHistory(EBallHandSide Side) const
	{
		return Side == EBallHandSide::Left ? LeftHistory : RightHistory;
	}
	static bool FindUniqueGrips(AActor *Actor, UMotionControllerComponent *&Left, UMotionControllerComponent *&Right);

  private:
	friend class UBallSimulationSubsystem;
	UPROPERTY()
	TObjectPtr<UCameraComponent> HeadCamera;
	UPROPERTY()
	TObjectPtr<USceneComponent> Origin;
	FBallHandPoseHistory LeftHistory, RightHistory;
	uint32 ParticipantId = 0, OriginRevision = 1, Sequence = 0;
	uint32 ConfigurationVersion = 1;
	FDelegateHandle RecenterHandle;
	TWeakObjectPtr<UBallSimulationSubsystem> System;
	FBallMotionSegment DebugSegments[2 * BallMotionLimits::SegmentsPerCollider];
	int32 DebugSegmentCount = 0;
	FTransform LastOrigin = FTransform::Identity;
	bool bHaveOrigin = false;
	FBallPalmSettings LastLeftPalm, LastRightPalm;
	FBallHandTrackingSettings LastTracking;
	bool bHaveSettings = false;
	bool bHadValidBinding = false;
	double LastWorldToMeters = 0;
	bool RefreshConfiguration();
	void DrawDiagnostics() const;
	bool IsHandDebugEnabled() const;
	void TickNetwork(bool Local, double Now);
	void ConfigureAuthority(bool Force = false);
	void SubmitCapturedPoses(double Now);
	void UpdateRemotePresentation(double Now);
	UPROPERTY(ReplicatedUsing = OnRep_AuthorityProfile) FBallWirePacket AuthorityProfilePacket;
	UPROPERTY(ReplicatedUsing = OnRep_RemotePoses) FBallWirePacket RemotePosePacket;
	UFUNCTION() void OnRep_AuthorityProfile();
	UFUNCTION() void OnRep_RemotePoses();
	UFUNCTION(Server, Unreliable) void ServerSubmitPoses(const FBallWirePacket &Packet);
	UFUNCTION(Client, Unreliable) void ClientClockProbe(uint32 Probe, uint32 ProfileVersion, double Sent);
	UFUNCTION(Server, Unreliable)
	void ServerClockReply(uint32 Probe, uint32 ProfileVersion, double Received, double Sent);
	UFUNCTION(Client, Unreliable)
	void ClientClockEstimate(uint32 ProfileVersion, double Reference, double Offset, double Drift, double RTT,
							 double Jitter, double Uncertainty, uint32 Version, bool Ready);
	UFUNCTION(Server, Unreliable) void ServerRequestBaseline();
	UFUNCTION(Server, Reliable) void ServerRequestOrigin(FVector Location, FRotator Rotation, uint32 ProfileVersion);
	FBallPeerProfile AuthorityProfile;
	FBallPeerProfile PreviousProfiles[16];
	int32 ProfileHead = 0;
	TUniquePtr<FBallPoseAdmission> Admission;
	FBallNetworkClock ServerClock;
	FBallClockEstimate ReceivedClock;
	bool bClockReady = false;
	struct FProbe
	{
		uint32 Sequence = 0;
		double Sent = 0;
	};
	FProbe Probes[8];
	uint32 ProbeSequence = 0, PacketSequence = 0, RemoteSequence = 0;
	uint32 ClockFailures = 0;
	double NextProbe = 0, NextPose = 0, NextRemote = 0, NextBaseline = 0, NextOriginRequest = 0;
	FBallTokenBucket DecodeRate, ClockRate, BaselineRate, OriginRate;
	struct FRemoteHandBuffer
	{
		FBallHandObservation Samples[32];
		uint32 Generation = 0;
	};
	FRemoteHandBuffer RemoteHands[2];
	int32 RemoteCounts[2] = {};
	double RemoteReceiveSeconds = 0;
	uint64 SentPoseBytes = 0, SentPosePackets = 0, RemoteBytes = 0, RejectedDecode = 0;
	EBallInputDisposition LastInputDisposition = EBallInputDisposition::Accepted;
};
