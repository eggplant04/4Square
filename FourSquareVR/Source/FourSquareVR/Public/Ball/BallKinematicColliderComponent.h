#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Ball/BallHandPoseHistory.h"
#include "Ball/BallRewindableCollider.h"
#include "BallKinematicColliderComponent.generated.h"

UENUM(BlueprintType)
enum class EBallKinematicShape : uint8
{
	Sphere,
	TranslatingBox
};
class UBallSimulationSubsystem;

/** Explicit opt-in historical dynamic proxy: sphere or fixed-orientation translating box. */
UCLASS(ClassGroup = (Ball), meta = (BlueprintSpawnableComponent))
class FOURSQUAREVR_API UBallKinematicColliderComponent : public UActorComponent, public IBallRewindableCollider
{
	GENERATED_BODY()
  public:
	UBallKinematicColliderComponent();
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void TickComponent(float Delta, ELevelTick Type, FActorComponentTickFunction *Tick) override;
	virtual void ApplyWorldOffset(const FVector &Offset, bool bWorldShift) override;
	virtual bool AppendRecordedMotion(FBallMotionFrame &Frame, double StartSeconds) override;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Kinematic")
	TObjectPtr<USceneComponent> TrackedComponent;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Kinematic")
	EBallKinematicShape Shape = EBallKinematicShape::Sphere;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Kinematic")
	FBallPalmSettings Sphere;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Kinematic", meta = (ClampMin = "0.01"))
	FVector BoxHalfExtents = FVector(10);
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Kinematic", meta = (ClampMin = "1"))
	double MaximumSpeed = 2500;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Debug")
	bool bDebugDraw = false;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ball|Debug")
	EBallTrackingRejection Rejection = EBallTrackingRejection::WarmingUp;
	UFUNCTION(BlueprintCallable, Category = "Ball|Kinematic")
	void NotifyDiscontinuity();
	SIZE_T GetHistoryAllocatedBytes() const
	{
		return History.GetAllocatedBytes();
	}

  private:
	friend class UBallSimulationSubsystem;
	TWeakObjectPtr<UBallSimulationSubsystem> System;
	FBallHandPoseHistory History;
	uint32 SourceId = 0, Sequence = 0, OriginRevision = 1;
	uint32 ConfigurationVersion = 1;
	FBallPalmSettings LastSphere;
	FVector LastExtents = FVector::ZeroVector;
	double LastMaximumSpeed = 0;
	EBallKinematicShape LastShape = EBallKinematicShape::Sphere;
	bool bHaveSettings = false;
	bool bHadValidTarget = false;
	TWeakObjectPtr<USceneComponent> LastTrackedComponent;
	TWeakObjectPtr<USceneComponent> LastConfiguredTarget;
	FBallHandTrackingSettings Safety() const;
	bool RefreshConfiguration();
};
