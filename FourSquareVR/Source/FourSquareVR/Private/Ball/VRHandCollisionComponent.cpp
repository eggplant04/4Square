#include "Ball/VRHandCollisionComponent.h"
#include "Ball/BallSimulationSubsystem.h"
#include "MotionControllerComponent.h"
#include "Camera/CameraComponent.h"
#include "GameFramework/Pawn.h"
#include "Engine/Engine.h"
#include "Engine/World.h"
#include "GameFramework/WorldSettings.h"
#include "IXRTrackingSystem.h"
#include "Misc/CoreDelegates.h"
#include "HAL/IConsoleManager.h"
#include "HAL/PlatformTime.h"
#include "DrawDebugHelpers.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

static TAutoConsoleVariable<int32> CVarBallHandsDebug(
	TEXT("ball.DebugHands"), 0, TEXT("Draw recorded palm trajectories, velocities and rejection reasons."));
UVRHandCollisionComponent::UVRHandCollisionComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;
	SetIsReplicatedByDefault(false);
}
bool UVRHandCollisionComponent::FindUniqueGrips(AActor *Actor, UMotionControllerComponent *&Left,
												UMotionControllerComponent *&Right)
{
	Left = Right = nullptr;
	if (!IsValid(Actor))
	{
		return false;
	}
	TInlineComponentArray<UMotionControllerComponent *> Components;
	Actor->GetComponents(Components);
	int32 L = 0, R = 0;
	for (auto *C : Components)
	{
		if (C->GetTrackingMotionSource() == TEXT("LeftGrip"))
		{
			Left = C;
			++L;
		}
		if (C->GetTrackingMotionSource() == TEXT("RightGrip"))
		{
			Right = C;
			++R;
		}
	}
	return L == 1 && R == 1;
}
bool UVRHandCollisionComponent::RegisterMotionControllers(UMotionControllerComponent *Left,
														  UMotionControllerComponent *Right, UCameraComponent *Head,
														  USceneComponent *TrackingOrigin)
{
	if (!IsValid(Left) || !IsValid(Right) || !IsValid(Head) || !IsValid(TrackingOrigin) || Left == Right ||
		!Left->IsRegistered() || !Right->IsRegistered() || !Head->IsRegistered() || !TrackingOrigin->IsRegistered() ||
		Left->GetOwner() != GetOwner() || Right->GetOwner() != GetOwner() || Head->GetOwner() != GetOwner() ||
		TrackingOrigin->GetOwner() != GetOwner() || Left->GetTrackingMotionSource() != TEXT("LeftGrip") ||
		Right->GetTrackingMotionSource() != TEXT("RightGrip") || !Left->IsAttachedTo(TrackingOrigin) ||
		!Right->IsAttachedTo(TrackingOrigin) || !Head->IsAttachedTo(TrackingOrigin))
	{
		return false;
	}
	if (LeftGrip == Left && RightGrip == Right && HeadCamera == Head && Origin == TrackingOrigin)
	{
		return true;
	}
	if (LeftGrip)
	{
		RemoveTickPrerequisiteComponent(LeftGrip);
	}
	if (RightGrip)
	{
		RemoveTickPrerequisiteComponent(RightGrip);
	}
	if (HeadCamera)
	{
		RemoveTickPrerequisiteComponent(HeadCamera);
	}
	if (Origin)
	{
		RemoveTickPrerequisiteComponent(Origin);
	}
	LeftGrip = Left;
	RightGrip = Right;
	HeadCamera = Head;
	Origin = TrackingOrigin;
	AddTickPrerequisiteActor(GetOwner());
	AddTickPrerequisiteComponent(Left);
	AddTickPrerequisiteComponent(Right);
	AddTickPrerequisiteComponent(Head);
	AddTickPrerequisiteComponent(TrackingOrigin);
	NotifyTrackingDiscontinuity();
	bHadValidBinding = true;
	return true;
}
bool UVRHandCollisionComponent::DiscoverGripControllers()
{
	UMotionControllerComponent *L = nullptr, *R = nullptr;
	if (!FindUniqueGrips(GetOwner(), L, R))
	{
		return false;
	}
	TInlineComponentArray<UCameraComponent *> Cameras;
	GetOwner()->GetComponents(Cameras);
	return Cameras.Num() == 1 && RegisterMotionControllers(L, R, Cameras[0], L->GetAttachParent());
}
bool UVRHandCollisionComponent::AreControllersRegistered() const
{
	return IsValid(LeftGrip) && IsValid(RightGrip) && IsValid(HeadCamera) && IsValid(Origin) &&
		   LeftGrip->IsRegistered() && RightGrip->IsRegistered() && HeadCamera->IsRegistered() &&
		   Origin->IsRegistered();
}
void UVRHandCollisionComponent::BeginPlay()
{
	Super::BeginPlay();
	LeftHistory.Initialize();
	RightHistory.Initialize();
	DiscoverGripControllers();
	System = GetWorld()->GetSubsystem<UBallSimulationSubsystem>();
	if (System.IsValid())
	{
		ParticipantId = System->RegisterHandSource(this);
	}
	RecenterHandle =
		FCoreDelegates::VRHeadsetRecenter.AddUObject(this, &UVRHandCollisionComponent::NotifyTrackingDiscontinuity);
}
void UVRHandCollisionComponent::EndPlay(const EEndPlayReason::Type Reason)
{
	FCoreDelegates::VRHeadsetRecenter.Remove(RecenterHandle);
	if (System.IsValid())
	{
		System->UnregisterHandSource(this);
	}
	Super::EndPlay(Reason);
}
void UVRHandCollisionComponent::NotifyTrackingDiscontinuity()
{
	if (OriginRevision < MAX_uint32)
	{
		++OriginRevision;
	}
	LeftHistory.NotifyDiscontinuity();
	RightHistory.NotifyDiscontinuity();
	LeftRejection = RightRejection = EBallTrackingRejection::Discontinuity;
	LeftPalmVelocity = RightPalmVelocity = FVector::ZeroVector;
}
void UVRHandCollisionComponent::ApplyWorldOffset(const FVector &Offset, bool bWorldShift)
{
	Super::ApplyWorldOffset(Offset, bWorldShift);
	NotifyTrackingDiscontinuity();
}
void UVRHandCollisionComponent::TickComponent(float Delta, ELevelTick Type, FActorComponentTickFunction *Tick)
{
	Super::TickComponent(Delta, Type, Tick);
	const auto *Pawn = Cast<APawn>(GetOwner());
	const bool Local =
		bAllowScriptedTrackingForTests || (Pawn && Pawn->IsPlayerControlled() && Pawn->IsLocallyControlled());
	if (System.IsValid())
	{
		if (!Local && ParticipantId)
		{
			System->UnregisterHandSource(this);
			ParticipantId = 0;
			NotifyTrackingDiscontinuity();
		}
		if (Local && !ParticipantId)
		{
			ParticipantId = System->RegisterHandSource(this);
		}
	}
	CaptureTracking();
	DrawDiagnostics();
}
bool UVRHandCollisionComponent::RefreshConfiguration()
{
	if (bHaveSettings && (!(LastLeftPalm == LeftPalm) || !(LastRightPalm == RightPalm) || !(LastTracking == Tracking)))
	{
		if (ConfigurationVersion == MAX_uint32)
		{
			NotifyTrackingDiscontinuity();
			return false;
		}
		++ConfigurationVersion;
		NotifyTrackingDiscontinuity();
	}
	LastLeftPalm = LeftPalm;
	LastRightPalm = RightPalm;
	LastTracking = Tracking;
	bHaveSettings = true;
	return true;
}
void UVRHandCollisionComponent::CaptureTracking()
{
	TRACE_CPUPROFILER_EVENT_SCOPE(Handball_CaptureTrackedPalms);
	if (!System.IsValid() || !ParticipantId || Sequence == MAX_uint32 || OriginRevision == MAX_uint32)
	{
		return;
	}
	if (!RefreshConfiguration())
	{
		return;
	}
	if (!AreControllersRegistered() && !DiscoverGripControllers())
	{
		if (bHadValidBinding)
		{
			NotifyTrackingDiscontinuity();
			bHadValidBinding = false;
		}
		LeftRejection = RightRejection = EBallTrackingRejection::InvalidPose;
		return;
	}
	const auto *Pawn = Cast<APawn>(GetOwner());
	const bool Local =
		bAllowScriptedTrackingForTests || (Pawn && Pawn->IsPlayerControlled() && Pawn->IsLocallyControlled());
	const bool HeadTracked =
		!bRequireTrackedHead || bAllowScriptedTrackingForTests ||
		(GEngine && GEngine->XRSystem.IsValid() && GEngine->XRSystem->IsTracking(IXRTrackingSystem::HMDDeviceId));
	const FTransform CurrentOrigin = Origin->GetComponentTransform();
	if (bHaveOrigin && (!CurrentOrigin.GetLocation().Equals(LastOrigin.GetLocation(), 0.001) ||
						!CurrentOrigin.GetRotation().Equals(LastOrigin.GetRotation(), 1.e-6) ||
						!CurrentOrigin.GetScale3D().Equals(LastOrigin.GetScale3D(), 1.e-5)))
	{
		NotifyTrackingDiscontinuity();
	}
	LastOrigin = CurrentOrigin;
	bHaveOrigin = true;
	const double WorldToMeters = GetWorld()->GetWorldSettings()->WorldToMeters;
	if (LastWorldToMeters > 0 && WorldToMeters != LastWorldToMeters)
	{
		NotifyTrackingDiscontinuity();
	}
	LastWorldToMeters = WorldToMeters;
	FBallHandObservation Base;
	Base.CaptureSeconds = FPlatformTime::Seconds();
	Base.TimelineSeconds = System->GetTrackingTimelineSeconds();
	Base.Timebase = System->GetTrackingTimebase();
	Base.OriginRevision = OriginRevision;
	Base.Sequence = ++Sequence;
	Base.ParticipantId = ParticipantId;
	Base.ConfigurationVersion = ConfigurationVersion;
	Base.WorldToMeters = WorldToMeters;
	Base.HeadPosition = HeadCamera->GetComponentLocation();
	Base.HeadOrientation = HeadCamera->GetComponentQuat();
	Base.OriginPosition = Origin->GetComponentLocation();
	Base.OriginOrientation = Origin->GetComponentQuat();
	const bool UnitScale = Origin->GetComponentScale().Equals(FVector::OneVector, 1.e-5);
	auto Sample = [&](UMotionControllerComponent *Controller, EBallHandSide Side, bool Enabled,
					  FBallHandPoseHistory &History, EBallTrackingRejection &Reason, FVector &Velocity,
					  const FBallPalmSettings &Palm) {
		auto P = Base;
		P.Side = Side;
		P.ControllerPosition = Controller->GetComponentLocation();
		P.ControllerOrientation = Controller->GetComponentQuat();
		P.bTracked = Enabled && Local && HeadTracked && UnitScale &&
					 (bAllowScriptedTrackingForTests ||
					  (Controller->IsActive() && Controller->IsComponentTickEnabled() && Controller->IsTracked() &&
					   Controller->CurrentTrackingStatus == ETrackingStatus::Tracked));
		Reason = History.Add(P, Tracking);
		const auto *Last = History.GetLatest();
		Velocity = FVector::ZeroVector;
		if (Last && Last->bCollisionReady)
		{
			Velocity =
				Last->LinearVelocity +
				FVector::CrossProduct(Last->AngularVelocity, P.ControllerOrientation.RotateVector(Palm.CenterOffset));
		}
	};
	Sample(LeftGrip, EBallHandSide::Left, bEnableLeft, LeftHistory, LeftRejection, LeftPalmVelocity, LeftPalm);
	Sample(RightGrip, EBallHandSide::Right, bEnableRight, RightHistory, RightRejection, RightPalmVelocity, RightPalm);
}
void UVRHandCollisionComponent::BuildRecordedMotion(FBallMotionFrame &Frame, double Start)
{
	if (!RefreshConfiguration())
	{
		return;
	}
	const bool Debug = bDebugDraw || CVarBallHandsDebug.GetValueOnGameThread() != 0;
	DebugSegmentCount = 0;
	auto Build = [&](const FBallHandPoseHistory &H, const FBallPalmSettings &Palm, EBallHandSide Side,
					 EBallTrackingRejection &Reason) {
		const int32 Begin = Frame.SegmentCount;
		EBallTrackingRejection BuildReason;
		if (!H.BuildMotion(Start, Palm, Tracking, ParticipantId, Side, Frame, BuildReason, ConfigurationVersion))
		{
			Reason = BuildReason;
		}
		if (Debug)
		{
			for (int32 I = Begin; I < Frame.SegmentCount && DebugSegmentCount < UE_ARRAY_COUNT(DebugSegments); ++I)
			{
				DebugSegments[DebugSegmentCount++] = Frame.Segments[I];
			}
		}
	};
	Build(LeftHistory, LeftPalm, EBallHandSide::Left, LeftRejection);
	Build(RightHistory, RightPalm, EBallHandSide::Right, RightRejection);
}
void UVRHandCollisionComponent::DrawDiagnostics() const
{
	if (!bDebugDraw && CVarBallHandsDebug.GetValueOnGameThread() == 0)
	{
		return;
	}
	auto Draw = [&](const FBallHandPoseHistory &H, const FBallPalmSettings &Palm, const FVector &V,
					EBallTrackingRejection Reason, FColor Color) {
		const auto *Last = H.GetLatest();
		if (!Last)
		{
			return;
		}
		const FVector Center =
			Last->Pose.ControllerPosition + Last->Pose.ControllerOrientation.RotateVector(Palm.CenterOffset);
		DrawDebugSphere(GetWorld(), Center, Palm.Radius, 16,
						Last->bCollisionReady && Reason == EBallTrackingRejection::None ? Color : FColor::Red);
		DrawDebugLine(GetWorld(), Center, Center + V * .05, FColor::Yellow);
		DrawDebugString(GetWorld(), Center + FVector(0, 0, 12),
						FString::Printf(TEXT("Palm P%u G%u v=%.1f %s"), ParticipantId, Last->Generation, V.Size(),
										*UEnum::GetValueAsString(Reason)),
						nullptr, Color, 0);
	};
	Draw(LeftHistory, LeftPalm, LeftPalmVelocity, LeftRejection, FColor::Magenta);
	Draw(RightHistory, RightPalm, RightPalmVelocity, RightRejection, FColor::Cyan);
	for (int32 I = 0; I < DebugSegmentCount; ++I)
	{
		DrawDebugPoint(GetWorld(), DebugSegments[I].Start, 5, FColor::Blue);
		DrawDebugLine(GetWorld(), DebugSegments[I].Start, DebugSegments[I].End, FColor::Green);
	}
}
