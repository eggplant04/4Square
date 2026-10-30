#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Ball/BallHandPoseHistory.h"
#include "VRHandCollisionComponent.generated.h"

class UMotionControllerComponent;
class UCameraComponent;
class UBallSimulationSubsystem;

/** Samples existing Grip components. It creates no controller/input/collision bodies and sends no RPCs. */
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
};
