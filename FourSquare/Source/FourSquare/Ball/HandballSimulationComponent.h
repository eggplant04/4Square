
#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Engine/EngineTypes.h"
#include "HandballTypes.h"
#include "HandballSimulationComponent.generated.h"

class UHandballHandComponent;
struct FHitResult;

struct FHandballHandMotion
{
    FTransform Start;
    FTransform End;
    FVector Extent = FVector::ZeroVector;
    int32 TrackedHandIndex = INDEX_NONE;
};

struct FHandPoseSample
{
    double TimeSeconds = 0.0;
    FTransform Pose = FTransform::Identity;
    bool bValid = false;
};

UCLASS(ClassGroup = (Handball), meta = (BlueprintSpawnableComponent))
class FOURSQUARE_API UHandballSimulationComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    UHandballSimulationComponent();

    void RegisterHand(UHandballHandComponent* Hand);
    void UnregisterHand(UHandballHandComponent* Hand);

    bool TryGrabBall(UHandballHandComponent* Hand);
    void ReleaseBall(UHandballHandComponent* Hand);

    void SimulateStep(
        FHandballState& State,
        float DeltaTime,
        const TArray<FHandballHandMotion>& HandMotions
    );
    
    float GetMaxBallSpeed() const
    {
        return MaxBallSpeed;
    }

protected:
    virtual void BeginPlay() override;

    virtual void TickComponent(
        float DeltaTime,
        ELevelTick TickType,
        FActorComponentTickFunction* ThisTickFunction
    ) override;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Handball|Physics")
    float Gravity = -980.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Handball|Physics",
        meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float Restitution = 0.8f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Handball|Physics",
        meta = (ClampMin = "0.0"))
    float MinBounceSpeed = 30.0f;
    
    UPROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Handball|Physics", meta = (ClampMin = "0.0"))
    float MaxBallSpeed = 2500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Handball|Hands", meta = (ClampMin = "0.0"))
    float HandHitStrength = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Handball|Hands",
        meta = (ClampMin = "0.0", ClampMax = "1.0"))
    float HandDirectionInfluence = 0.35f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Handball|Hands", meta = (ClampMin = "0.0"))
    float GrabReach = 5.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Handball|Hands", meta = (ClampMin = "0.0"))
    float ThrowStrength = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Handball|Hands", meta = (ClampMin = "0.0"))
    float HandHitCooldown = 0.15f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Handball|Hands", meta = (ClampMin = "0.0"))
    float HandRearmDistance = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Handball|Tracking", meta = (ClampMin = "0.0"))
    float MaxHandTrackingSpeed = 1500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Handball|Tracking", meta = (ClampMin = "0.0"))
    float MaxHandAngularSpeed = 1800.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Handball|Tracking", meta = (ClampMin = "0.001"))
    float MaxHandSampleDelta = 0.1f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Handball|Tracking",
        meta = (ClampMin = "0.05"))
    float HandPoseHistorySeconds = 0.2f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite,
        Category = "Handball|Tracking",
        meta = (ClampMin = "0.01"))
    float ThrowVelocityWindow = 0.05f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Handball|Collision",
        meta = (ClampMin = "0.1"))
    float BallRadius = 3.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Handball|Collision")
    TEnumAsByte<ETraceTypeQuery> BarrierTraceChannel = TraceTypeQuery1;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Handball|Debug")
    bool bDebugRadius = false;

private:
    struct FTrackedHand
    {
        TWeakObjectPtr<UHandballHandComponent> Hand;
        FTransform LastRenderPose;
        FTransform LastSimPose;
        FTransform CurrentPose;
        float CooldownRemaining = 0.0f;
        bool bNeedsSeparation = false;

        FVector ThrowVelocity = FVector::ZeroVector;
        bool bMotionValid = false;
        TArray<FHandPoseSample> PoseHistory;
    };

    struct FHandHit
    {
        float Time = 1.0f;
        float Penetration = 0.0f;
        FVector Normal = FVector::ZeroVector;
        FVector HandVelocity = FVector::ZeroVector;
    };

    TArray<FTrackedHand> TrackedHands;

    TWeakObjectPtr<UHandballHandComponent> HoldingHand;
    FVector HeldLocalOffset = FVector::ZeroVector;

    void CaptureHandPose(
        FTrackedHand& Hand,
        double TimeSeconds
    );

    bool GetHandPoseAtTime(
        const FTrackedHand& Hand,
        double TimeSeconds,
        FTransform& OutPose
    ) const;

    FVector CalculateThrowVelocity(
        const FTrackedHand& Hand
    ) const;

    void ClampBallSpeed(FHandballState& State) const;

    bool FindBarrierHit(
        const FVector& Start,
        const FVector& End,
        FHitResult& OutHit
    ) const;

    bool FindHandHit(
        const FVector& Start,
        const FVector& End,
        const FHandballHandMotion& Motion,
        float Duration,
        FHandHit& OutHit
    ) const;

    void ResolveBounce(
        FHandballState& State,
        const FVector& Normal,
        const FVector& SurfaceVelocity
    ) const;

    void ResolveHandBounce(
        FHandballState& State,
        const FVector& Normal,
        const FVector& HandVelocity
    ) const;

    float AccumulatedTime = 0.0f;

    static constexpr float FixedTimeStep = 1.0f / 120.0f;
    static constexpr int32 MaxStepsPerFrame = 8;
};
