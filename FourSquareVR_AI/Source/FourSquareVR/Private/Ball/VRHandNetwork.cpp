#include "Ball/VRHandCollisionComponent.h"
#include "Ball/BallSimulationSubsystem.h"
#include "Ball/BallPalmDebug.h"
#include "MotionControllerComponent.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/PlayerController.h"
#include "GameFramework/WorldSettings.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"
#include "NavigationSystem.h"
#include "Net/UnrealNetwork.h"
#include "DrawDebugHelpers.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

void UVRHandCollisionComponent::GetLifetimeReplicatedProps(TArray<FLifetimeProperty> &OutLifetimeProps) const
{
	Super::GetLifetimeReplicatedProps(OutLifetimeProps);
	DOREPLIFETIME(UVRHandCollisionComponent, AuthorityProfilePacket);
	DOREPLIFETIME(UVRHandCollisionComponent, RemotePosePacket);
}
void UVRHandCollisionComponent::ConfigureAuthority(bool Force)
{
	if (!GetOwner()->HasAuthority() || !ParticipantId || !System.IsValid() || !AreControllersRegistered())
		return;
	const bool Initial = !AuthorityProfile.IsValid();
	const bool Context = Initial || AuthorityProfile.Timebase != System->GetTrackingTimebase() ||
						 AuthorityProfile.Epoch != System->GetEpoch();
	if (!Force && !Context)
		return;
	if (!LeftPalm.IsValid() || !RightPalm.IsValid() || !Tracking.IsValid() || !Transport.IsValid())
		return;
	if (!Initial)
		PreviousProfiles[ProfileHead++ % UE_ARRAY_COUNT(PreviousProfiles)] = AuthorityProfile;
	FBallPeerProfile P;
	P.Epoch = System->GetEpoch();
	P.Participant = ParticipantId;
	P.Version = Initial ? 1 : AuthorityProfile.Version + 1;
	P.Timebase = System->GetTrackingTimebase();
	P.EffectiveTimelineSeconds = System->MapMonotonicToTimeline(FPlatformTime::Seconds());
	P.EffectiveFrame = System->GetSimulationFrame();
	P.OriginRevision = Initial ? 1 : AuthorityProfile.OriginRevision + 1;
	P.Left = LeftPalm;
	P.Right = RightPalm;
	P.bEnableLeft = bEnableLeft;
	P.bEnableRight = bEnableRight;
	P.Tracking = Tracking;
	P.Transport = Transport;
	P.Origin = Origin->GetComponentLocation();
	P.OriginRotation = Origin->GetComponentQuat();
	P.WorldToMeters = GetWorld()->GetWorldSettings()->WorldToMeters;
	if (!P.IsValid() || !FBallNetworkCodec::WriteProfile(P, AuthorityProfilePacket))
		return;
	AuthorityProfile = P;
	ConfigurationVersion = P.Version;
	LastLeftPalm = LeftPalm;
	LastRightPalm = RightPalm;
	LastTracking = Tracking;
	bHaveSettings = true;
	if (!Admission)
	{
		Admission = MakeUnique<FBallPoseAdmission>();
		Admission->Initialize(P);
	}
	else
		Admission->ChangeProfile(P);
	ServerClock.Reset();
	ReceivedClock = {};
	bClockReady = false;
	NextProbe = 0;
	ClockFailures = 0;
	NotifyTrackingDiscontinuity();
	GetOwner()->ForceNetUpdate();
}
void UVRHandCollisionComponent::OnRep_AuthorityProfile()
{
	FBallPeerProfile P;
	if (!FBallNetworkCodec::ReadProfile(AuthorityProfilePacket, P))
	{
		++RejectedDecode;
		return;
	}
	if (AuthorityProfile.IsValid() && !FBallFinalityPolicy::IsSequenceNewer(P.Version, AuthorityProfile.Version))
		return;
	AuthorityProfile = P;
	ParticipantId = P.Participant;
	ConfigurationVersion = P.Version;
	LeftPalm = P.Left;
	RightPalm = P.Right;
	Tracking = P.Tracking;
	Transport = P.Transport;
	bEnableLeft = P.bEnableLeft;
	bEnableRight = P.bEnableRight;
	LastLeftPalm = LeftPalm;
	LastRightPalm = RightPalm;
	LastTracking = Tracking;
	bHaveSettings = true;
	ReceivedClock = {};
	bClockReady = false;
	RemoteCounts[0] = RemoteCounts[1] = 0;
	const auto *Pawn = Cast<APawn>(GetOwner());
	if (Pawn && Pawn->IsLocallyControlled())
	{
		LeftHistory.Initialize();
		RightHistory.Initialize();
	}
	NotifyTrackingDiscontinuity();
}
void UVRHandCollisionComponent::TickNetwork(bool Local, double Now)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Handball_NetworkHands);
	if (!System.IsValid())
		System = GetWorld()->GetSubsystem<UBallSimulationSubsystem>();
	if (!AreControllersRegistered() && !DiscoverGripControllers())
		return;
	if (GetOwner()->HasAuthority())
	{
		if (!Transport.IsValid())
			return;
		if (!ParticipantId && System.IsValid())
			ParticipantId = System->RegisterHandSource(this);
		if (AuthorityProfile.IsValid() &&
			(!(AuthorityProfile.Left == LeftPalm) || !(AuthorityProfile.Right == RightPalm) ||
			 !(AuthorityProfile.Tracking == Tracking)))
		{
			if (!System->RebuildStaticEnvironment())
				return;
			ConfigureAuthority(true);
		}
		else
			ConfigureAuthority();
		if (!AuthorityProfile.IsValid() || !Admission)
			return;
		if (Local)
		{
			if (!LeftHistory.GetAllocatedBytes())
			{
				LeftHistory.Initialize();
				RightHistory.Initialize();
			}
			// Listen host goes through the identical quantized codec and server validation path.
			ReceivedClock = {};
			ReceivedClock.Version = 1;
			ReceivedClock.Uncertainty = 0;
			bClockReady = true;
			CaptureTracking();
			SubmitCapturedPoses(Now);
		}
		else if (Now >= NextProbe)
		{
			const uint32 Probe = ++ProbeSequence;
			Probes[Probe % UE_ARRAY_COUNT(Probes)] = {Probe, Now};
			ClientClockProbe(Probe, AuthorityProfile.Version, Now);
			NextProbe = Now + (ServerClock.IsReady() ? .5 : .1);
		}
		if (Now >= NextRemote)
		{
			FBallPoseBatch B;
			B.Epoch = AuthorityProfile.Epoch;
			B.Timebase = AuthorityProfile.Timebase;
			B.ProfileVersion = AuthorityProfile.Version;
			B.OriginRevision = AuthorityProfile.OriginRevision;
			B.Sequence = ++RemoteSequence;
			B.SentSeconds = Now;
			for (uint8 Side = 0; Side < 2; ++Side)
			{
				const auto V = Admission->Hand(EBallHandSide(Side)).View();
				if (!V.Num())
					continue;
				const auto &S = V.Last();
				auto P = S.Pose;
				P.CaptureSeconds = Now - (System->GetTrackingTimelineSeconds() - P.TimelineSeconds);
				B.Generations[B.Count] = S.Generation;
				B.Samples[B.Count++] = P;
			}
			if (B.Count && FBallNetworkCodec::WritePoses(B, RemotePosePacket, true))
				RemoteBytes += RemotePosePacket.Bytes.Num();
			if (NextRemote <= 0)
				NextRemote = Now;
			NextRemote += (FMath::FloorToDouble(FMath::Max(0.0, Now - NextRemote) * Transport.RemotePoseRate) + 1) /
						  Transport.RemotePoseRate;
		}
	}
	else if (Local)
	{
		if (!AuthorityProfile.IsValid())
			return;
		if (!LeftHistory.GetAllocatedBytes())
		{
			LeftHistory.Initialize();
			RightHistory.Initialize();
		}
		// Client property edits cannot alter the approved collider/validation profile.
		LeftPalm = AuthorityProfile.Left;
		RightPalm = AuthorityProfile.Right;
		Tracking = AuthorityProfile.Tracking;
		Transport = AuthorityProfile.Transport;
		bEnableLeft = AuthorityProfile.bEnableLeft;
		bEnableRight = AuthorityProfile.bEnableRight;
		CaptureTracking();
		if ((!Origin->GetComponentLocation().Equals(AuthorityProfile.Origin, Transport.OriginPositionTolerance) ||
			 Origin->GetComponentQuat().AngularDistance(AuthorityProfile.OriginRotation) >
				 Transport.OriginAngularTolerance) &&
			Now >= NextOriginRequest)
		{
			ServerRequestOrigin(Origin->GetComponentLocation(), Origin->GetComponentRotation(),
								AuthorityProfile.Version);
			NextOriginRequest = Now + .5;
		}
		if (bClockReady)
			SubmitCapturedPoses(Now);
	}
	else
		UpdateRemotePresentation(Now);
}
void UVRHandCollisionComponent::SubmitCapturedPoses(double Now)
{
	if (Now < NextPose || !AuthorityProfile.IsValid())
		return;
	FBallPoseBatch B;
	B.Epoch = AuthorityProfile.Epoch;
	B.Timebase = AuthorityProfile.Timebase;
	B.ProfileVersion = AuthorityProfile.Version;
	B.OriginRevision = AuthorityProfile.OriginRevision;
	B.Sequence = ++PacketSequence;
	B.SentSeconds = Now;
	for (uint8 Side = 0; Side < 2; ++Side)
	{
		const auto &H = GetHandHistory(EBallHandSide(Side));
		// Six samples per hand redundantly cover short losses; each retains its actual capture time.
		for (int32 I = FMath::Max(0, H.Num() - 6); I < H.Num(); ++I)
		{
			const auto *S = H.GetSample(I);
			if (!S || S->Pose.ConfigurationVersion != AuthorityProfile.Version)
				continue;
			B.Generations[B.Count] = S->Generation;
			B.Samples[B.Count] = S->Pose;
			B.Samples[B.Count].Timebase = B.Timebase;
			B.Samples[B.Count].OriginRevision = B.OriginRevision;
			++B.Count;
		}
	}
	FBallWirePacket Wire;
	B.SentSeconds = FPlatformTime::Seconds(); // Send observation follows capture/collection, including hitches.
	if (B.Count && FBallNetworkCodec::WritePoses(B, Wire))
	{
		++SentPosePackets;
		SentPoseBytes += Wire.Bytes.Num();
		if (GetOwner()->HasAuthority())
			ServerSubmitPoses_Implementation(Wire);
		else
			ServerSubmitPoses(Wire);
	}
	// Server-issued estimate, bounded transport-only adaptation. Physics and history policy are unchanged.
	const double Congestion = FMath::Clamp(ReceivedClock.Jitter / .02, 0.0, 1.0);
	const double Rate = FMath::Lerp(Transport.PoseRate, Transport.MinimumPoseRate, Congestion);
	if (NextPose <= 0)
		NextPose = Now;
	NextPose += (FMath::FloorToDouble(FMath::Max(0.0, Now - NextPose) * Rate) + 1) / Rate;
}
void UVRHandCollisionComponent::ServerSubmitPoses_Implementation(const FBallWirePacket &Wire)
{
	const double Now = FPlatformTime::Seconds();
	const auto *Pawn = Cast<APawn>(GetOwner());
	if (!Pawn || !Pawn->IsPlayerControlled() || !Admission || !System.IsValid() ||
		!DecodeRate.Consume(Now, Transport.MaximumRPCsPerSecond))
	{
		++RejectedDecode;
		return;
	}
	FBallPoseBatch B;
	const double Begin = FPlatformTime::Seconds();
	if (!FBallNetworkCodec::ReadPoses(Wire, B))
	{
		++RejectedDecode;
		return;
	}
	const double DecodeUs = (FPlatformTime::Seconds() - Begin) * 1.e6;
	const bool Host = Pawn->IsLocallyControlled();
	FBallClockEstimate E = Host ? ReceivedClock : ServerClock.GetEstimate(Now);
	LastInputDisposition =
		Admission->Admit(B, E, Host || ServerClock.IsReady(), Now, System->MapMonotonicToTimeline(Now),
						 System->GetSimulationFrame() / System->GetFrequency());
	Admission->RecordTransport(Wire.Bytes.Num(), DecodeUs);
}
void UVRHandCollisionComponent::ClientClockProbe_Implementation(uint32 Probe, uint32 Version, double Sent)
{
	const double Received = FPlatformTime::Seconds();
	if (Version != AuthorityProfile.Version || !FMath::IsFinite(Sent))
		return;
	ServerClockReply(Probe, Version, Received, FPlatformTime::Seconds());
}
void UVRHandCollisionComponent::ServerClockReply_Implementation(uint32 Probe, uint32 Version, double Received,
																double Sent)
{
	const double Now = FPlatformTime::Seconds();
	auto &P = Probes[Probe % UE_ARRAY_COUNT(Probes)];
	if (Version != AuthorityProfile.Version || !Probe || P.Sequence != Probe || !ClockRate.Consume(Now, 12))
		return;
	const double Start = P.Sent;
	P = {};
	if (!ServerClock.Observe(Start, Received, Sent, Now, Transport.MaximumClockDriftPPM))
	{
		// An explicit per-source epoch replaces an unusable clock; old accepted timestamps remain frozen.
		if (++ClockFailures >= 4)
			ConfigureAuthority(true);
		return;
	}
	ClockFailures = 0;
	const auto E = ServerClock.GetEstimate(Now);
	ClientClockEstimate(Version, E.ReferenceLocal, E.RemoteMinusLocal, E.Drift, E.RTT, E.Jitter, E.Uncertainty,
						E.Version, ServerClock.IsReady());
}
const FBallPeerProfile *UVRHandCollisionComponent::FindHistoricalProfile(uint32 Version) const
{
	if (AuthorityProfile.Version == Version && AuthorityProfile.IsValid())
		return &AuthorityProfile;
	for (const auto &P : PreviousProfiles)
		if (P.Version == Version && P.IsValid())
			return &P;
	return nullptr;
}
void UVRHandCollisionComponent::ClientClockEstimate_Implementation(uint32 Profile, double Reference, double Offset,
																   double Drift, double RTT, double Jitter,
																   double Uncertainty, uint32 Version, bool Ready)
{
	FBallClockEstimate E;
	E.ReferenceLocal = Reference;
	E.RemoteMinusLocal = Offset;
	E.Drift = Drift;
	E.RTT = RTT;
	E.Jitter = Jitter;
	E.Uncertainty = Uncertainty;
	E.Version = Version;
	if (Profile != AuthorityProfile.Version || !E.IsValid() ||
		(ReceivedClock.Version && !FBallFinalityPolicy::IsSequenceNewer(Version, ReceivedClock.Version)))
		return;
	ReceivedClock = E;
	bClockReady = Ready;
}
double UVRHandCollisionComponent::GetEstimatedServerSeconds(double Now) const
{
	return bClockReady && ReceivedClock.IsValid() ? ReceivedClock.RemoteToLocal(Now) : Now;
}
void UVRHandCollisionComponent::OnRep_RemotePoses()
{
	const auto *Pawn = Cast<APawn>(GetOwner());
	if (Pawn && Pawn->IsLocallyControlled())
		return;
	FBallPoseBatch B;
	if (!FBallNetworkCodec::ReadPoses(RemotePosePacket, B, true) || B.ProfileVersion != AuthorityProfile.Version)
	{
		++RejectedDecode;
		return;
	}
	for (int32 I = 0; I < B.Count; ++I)
	{
		const auto &P = B.Samples[I];
		const uint8 Side = uint8(P.Side);
		int32 &Count = RemoteCounts[Side];
		if (RemoteHands[Side].Generation != B.Generations[I])
		{
			Count = 0;
			RemoteHands[Side].Generation = B.Generations[I];
		}
		if (Count && P.CaptureSeconds <= RemoteHands[Side].Samples[Count - 1].CaptureSeconds)
			continue;
		if (Count == 32)
		{
			for (int32 J = 1; J < 32; ++J)
				RemoteHands[Side].Samples[J - 1] = RemoteHands[Side].Samples[J];
			--Count;
		}
		RemoteHands[Side].Samples[Count++] = P;
	}
	RemoteReceiveSeconds = FPlatformTime::Seconds();
}
void UVRHandCollisionComponent::UpdateRemotePresentation(double Now)
{
	if (!AuthorityProfile.IsValid())
		return;
	LeftGrip->SetComponentTickEnabled(false);
	RightGrip->SetComponentTickEnabled(false);
	HeadCamera->bLockToHmd = false;
	double ServerNow = 0;
	double Jitter = 0;
	bool Clock = false;
	if (APlayerController *PC = GetWorld()->GetFirstPlayerController())
	{
		if (APawn *OwnPawn = PC->GetPawn())
		{
			if (UVRHandCollisionComponent *Hands = OwnPawn->FindComponentByClass<UVRHandCollisionComponent>())
			{
				if (Hands->bClockReady)
				{
					ServerNow = Hands->GetEstimatedServerSeconds(Now);
					Jitter = Hands->ReceivedClock.Jitter;
					Clock = true;
				}
			}
		}
	}
	for (uint8 Side = 0; Side < 2; ++Side)
	{
		const int32 N = RemoteCounts[Side];
		if (!N)
			continue;
		const auto &Last = RemoteHands[Side].Samples[N - 1];
		auto P = Last;
		const double Time = (Clock ? ServerNow : Last.CaptureSeconds + (Now - RemoteReceiveSeconds)) -
							FMath::Clamp(Transport.VisualDelay + Jitter * 2, Transport.VisualDelay, .12);
		for (int32 I = 1; I < N; ++I)
			if (Time <= RemoteHands[Side].Samples[I].CaptureSeconds)
			{
				const auto &A = RemoteHands[Side].Samples[I - 1], &B = RemoteHands[Side].Samples[I];
				const double Alpha =
					FMath::Clamp((Time - A.CaptureSeconds) / (B.CaptureSeconds - A.CaptureSeconds), 0.0, 1.0);
				P = A;
				P.ControllerPosition = FMath::Lerp(A.ControllerPosition, B.ControllerPosition, Alpha);
				P.ControllerOrientation =
					FQuat::Slerp(A.ControllerOrientation, B.ControllerOrientation, Alpha).GetNormalized();
				P.HeadPosition = FMath::Lerp(A.HeadPosition, B.HeadPosition, Alpha);
				P.HeadOrientation = FQuat::Slerp(A.HeadOrientation, B.HeadOrientation, Alpha).GetNormalized();
				break;
			}
		UMotionControllerComponent *Grip = Side ? RightGrip.Get() : LeftGrip.Get();
		Grip->SetWorldLocationAndRotation(P.ControllerPosition, P.ControllerOrientation);
		Grip->SetVisibility(P.bTracked && Now - RemoteReceiveSeconds < .5, true);
		HeadCamera->SetWorldLocationAndRotation(P.HeadPosition, P.HeadOrientation);
		if (IsHandDebugEnabled())
		{
			const auto &Palm = Side ? RightPalm : LeftPalm;
			const auto T = Palm.WorldTransform(P.ControllerPosition, P.ControllerOrientation);
			if (Palm.Shape == EBallPalmShape::Sphere)
				DrawDebugSphere(GetWorld(), T.GetLocation(), Palm.Radius, 16, FColor::Cyan);
			else
				FBallPalmDebug::Draw(GetWorld(), T, Palm.BoxDimensions * .5, Palm.RoundRadius, FColor::Cyan);
		}
	}
}
void UVRHandCollisionComponent::RequestAuthoritativeBaseline()
{
	const double Now = FPlatformTime::Seconds();
	if (Now < NextBaseline)
		return;
	NextBaseline = Now + 1;
	ServerRequestBaseline();
}
void UVRHandCollisionComponent::ServerRequestBaseline_Implementation()
{
	if (System.IsValid() && BaselineRate.Consume(FPlatformTime::Seconds(), 1))
		System->ForceNetworkBaselines();
}
bool UVRHandCollisionComponent::SetServerPalmConfiguration(const FBallPalmSettings &L, const FBallPalmSettings &R)
{
	if (GetNetMode() == NM_Client || !L.IsValid() || !R.IsValid() || !System.IsValid())
		return false;
	LeftPalm = L;
	RightPalm = R;
	// Explicit geometry barrier: existing numeric world/ball histories are invalidated together.
	if (!System->RebuildStaticEnvironment())
		return false;
	ConfigureAuthority(true);
	System->ForceNetworkBaselines();
	return true;
}
bool UVRHandCollisionComponent::AuthorizeTrackingOrigin(FVector Location, FRotator Rotation)
{
	if (GetNetMode() == NM_Client || Location.ContainsNaN() || Rotation.ContainsNaN() || !AreControllersRegistered())
		return false;
	const FVector Delta = Location - Origin->GetComponentLocation();
	GetOwner()->AddActorWorldOffset(Delta, false);
	Origin->SetWorldLocationAndRotation(Location, Rotation);
	// Per-source continuity boundary; previously recorded numeric frames keep their old origin/configuration.
	ConfigureAuthority(true);
	return true;
}
void UVRHandCollisionComponent::ServerRequestOrigin_Implementation(FVector Location, FRotator Rotation, uint32 Version)
{
	if (!bAllowValidatedOriginRequests || Version != AuthorityProfile.Version || Location.ContainsNaN() ||
		Rotation.ContainsNaN() || !OriginRate.Consume(FPlatformTime::Seconds(), 2) ||
		FVector::Dist(Location, AuthorityProfile.Origin) > 600 || FMath::Abs(Rotation.Pitch) > .1 ||
		FMath::Abs(Rotation.Roll) > .1)
		return;
	// Same-position snap turns/recentering may be authorized without a navigation mesh.
	if (FVector::Dist(Location, AuthorityProfile.Origin) > 1)
	{
		auto *Nav = FNavigationSystem::GetCurrent<UNavigationSystemV1>(GetWorld());
		FNavLocation Projected;
		if (!Nav || !Nav->ProjectPointToNavigation(Location, Projected, FVector(25, 25, 100)) ||
			FVector::Dist(Location, Projected.Location) > 25)
			return;
	}
	AuthorizeTrackingOrigin(Location, Rotation);
}
FString UVRHandCollisionComponent::GetNetworkDiagnostics() const
{
	const double Now = FPlatformTime::Seconds();
	const APawn *Pawn = Cast<APawn>(GetOwner());
	const bool Host = GetOwner()->HasAuthority() && Pawn && Pawn->IsLocallyControlled();
	const auto E = GetOwner()->HasAuthority() && !Host ? ServerClock.GetEstimate(Now) : ReceivedClock;
	const auto S = Admission ? Admission->Statistics : FBallNetworkStatistics();
	const bool Ready = GetOwner()->HasAuthority() && !Host ? ServerClock.IsReady() : bClockReady;
	return FString::Printf(
		TEXT("Participant=%u Profile=%u ClockReady=%d RTTms=%.3f Jitterms=%.3f Uncertaintyms=%.3f DriftPPM=%.1f "
			 "SentPackets=%llu PoseBytes=%llu Accepted=%llu Duplicate=%llu Rejected=%llu DecodeRejected=%llu Gaps=%llu "
			 "Disposition=%d HistoryBytes=%llu DecodeUs=%.3f AdmissionUs=%.3f"),
		ParticipantId, AuthorityProfile.Version, Ready, E.RTT * 1000, E.Jitter * 1000, E.Uncertainty * 1000,
		E.Drift * 1.e6, SentPosePackets, SentPoseBytes, S.Accepted, S.Duplicates, S.Rejected, RejectedDecode,
		S.SequenceGaps, int32(LastInputDisposition),
		uint64(GetHistoryAllocatedBytes() + (Admission ? Admission->GetAllocatedBytes() : 0)), S.DecodeMicroseconds,
		S.AdmissionMicroseconds);
}
