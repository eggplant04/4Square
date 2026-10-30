#include "Ball/BallKinematicColliderComponent.h"
#include "Ball/BallSimulationSubsystem.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"
#include "DrawDebugHelpers.h"

UBallKinematicColliderComponent::UBallKinematicColliderComponent()
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = true;
	PrimaryComponentTick.TickGroup = TG_PostPhysics;
	SetIsReplicatedByDefault(false);
}
FBallHandTrackingSettings UBallKinematicColliderComponent::Safety() const
{
	FBallHandTrackingSettings S;
	S.MaximumSpeed = MaximumSpeed;
	S.MaximumHeadSpeed = 1.e12;
	S.MaximumReach = 1.e12;
	return S;
}
void UBallKinematicColliderComponent::BeginPlay()
{
	Super::BeginPlay();
	History.Initialize();
	if (!TrackedComponent)
	{
		TrackedComponent = GetOwner()->GetRootComponent();
	}
	AddTickPrerequisiteActor(GetOwner());
	if (TrackedComponent)
	{
		AddTickPrerequisiteComponent(TrackedComponent);
	}
	System = GetWorld()->GetSubsystem<UBallSimulationSubsystem>();
	if (System.IsValid())
	{
		SourceId = System->RegisterKinematicSource(this);
	}
}
void UBallKinematicColliderComponent::EndPlay(const EEndPlayReason::Type Reason)
{
	if (System.IsValid())
	{
		System->UnregisterKinematicSource(this);
	}
	Super::EndPlay(Reason);
}
void UBallKinematicColliderComponent::NotifyDiscontinuity()
{
	if (OriginRevision < MAX_uint32)
	{
		++OriginRevision;
	}
	History.NotifyDiscontinuity();
}
void UBallKinematicColliderComponent::ApplyWorldOffset(const FVector &Offset, bool bWorldShift)
{
	Super::ApplyWorldOffset(Offset, bWorldShift);
	NotifyDiscontinuity();
}
bool UBallKinematicColliderComponent::RefreshConfiguration()
{
	if (bHaveSettings && (!(LastSphere == Sphere) || LastExtents != BoxHalfExtents || LastShape != Shape ||
						  LastMaximumSpeed != MaximumSpeed || LastConfiguredTarget.Get() != TrackedComponent))
	{
		if (ConfigurationVersion == MAX_uint32)
		{
			NotifyDiscontinuity();
			return false;
		}
		++ConfigurationVersion;
		NotifyDiscontinuity();
	}
	LastSphere = Sphere;
	LastExtents = BoxHalfExtents;
	LastShape = Shape;
	LastMaximumSpeed = MaximumSpeed;
	LastConfiguredTarget = TrackedComponent;
	bHaveSettings = true;
	return true;
}
void UBallKinematicColliderComponent::TickComponent(float Delta, ELevelTick Type, FActorComponentTickFunction *Tick)
{
	Super::TickComponent(Delta, Type, Tick);
	if (!System.IsValid() || !SourceId || Sequence == MAX_uint32 || OriginRevision == MAX_uint32)
	{
		return;
	}
	if (!IsValid(TrackedComponent))
	{
		if (bHadValidTarget)
		{
			NotifyDiscontinuity();
			bHadValidTarget = false;
		}
		Rejection = EBallTrackingRejection::Untracked;
		return;
	}
	bHadValidTarget = true;
	if (!RefreshConfiguration())
	{
		return;
	}
	if (LastTrackedComponent.Get() != TrackedComponent)
	{
		if (LastTrackedComponent.IsValid())
		{
			RemoveTickPrerequisiteComponent(LastTrackedComponent.Get());
		}
		AddTickPrerequisiteComponent(TrackedComponent);
		LastTrackedComponent = TrackedComponent;
	}
	FBallHandObservation P;
	P.CaptureSeconds = FPlatformTime::Seconds();
	P.TimelineSeconds = System->GetTrackingTimelineSeconds();
	P.Timebase = System->GetTrackingTimebase();
	P.Sequence = ++Sequence;
	P.OriginRevision = OriginRevision;
	P.ParticipantId = SourceId;
	P.ConfigurationVersion = ConfigurationVersion;
	P.ControllerPosition = P.HeadPosition = TrackedComponent->GetComponentLocation();
	P.ControllerOrientation = TrackedComponent->GetComponentQuat();
	const auto *Primitive = Cast<UPrimitiveComponent>(TrackedComponent);
	P.bTracked = TrackedComponent->Mobility == EComponentMobility::Movable &&
				 TrackedComponent->GetComponentScale().Equals(FVector::OneVector, 1.e-5) &&
				 (!Primitive || !Primitive->IsSimulatingPhysics());
	const auto *Last = History.GetLatest();
	bool Unsupported = false;
	if (Shape == EBallKinematicShape::TranslatingBox && Last && P.Timebase == Last->Pose.Timebase &&
		P.ControllerOrientation.AngularDistance(Last->Pose.ControllerOrientation) > 1.e-6)
	{
		P.bTracked = false;
		Unsupported = true;
	}
	Rejection = History.Add(P, Safety());
	if (Unsupported)
	{
		Rejection = EBallTrackingRejection::UnsupportedRotation;
	}
	if (bDebugDraw)
	{
		const FVector C = P.ControllerPosition + P.ControllerOrientation.RotateVector(Sphere.CenterOffset);
		if (Shape == EBallKinematicShape::Sphere)
		{
			DrawDebugSphere(GetWorld(), C, Sphere.Radius, 16, FColor::Orange);
		}
		else
		{
			DrawDebugBox(GetWorld(), C, BoxHalfExtents, P.ControllerOrientation, FColor::Orange);
		}
		DrawDebugString(GetWorld(), C + FVector(0, 0, 15), UEnum::GetValueAsString(Rejection), nullptr, FColor::Orange,
						0);
	}
}
bool UBallKinematicColliderComponent::AppendRecordedMotion(FBallMotionFrame &F, double Start)
{
	if (!RefreshConfiguration())
	{
		return false;
	}
	const int32 Before = F.ColliderCount;
	EBallTrackingRejection R;
	if (!History.BuildMotion(Start, Sphere, Safety(), SourceId, EBallHandSide::Left, F, R, ConfigurationVersion))
	{
		Rejection = R;
		return false;
	}
	if (F.ColliderCount > Before)
	{
		auto &C = F.Colliders[Before];
		C.Id.Value = BallMotionLimits::KinematicNamespace | SourceId;
		C.Category = EBallContactCategory::Kinematic;
		C.Participant = 0;
		C.Shape = Shape == EBallKinematicShape::Sphere ? EBallMotionShape::Sphere : EBallMotionShape::TranslatingBox;
		C.HalfExtents = BoxHalfExtents;
		if (!F.IsValid())
		{
			F.SegmentCount = C.FirstSegment;
			F.ColliderCount = Before;
			Rejection = EBallTrackingRejection::InvalidPose;
			return false;
		}
	}
	return true;
}
