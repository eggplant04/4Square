
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "HandballTypes.h"
#include "Handball.generated.h"

class USceneComponent;
class UStaticMeshComponent;
class UHandballSimulationComponent;
class UMaterialInstanceDynamic;

UCLASS()
class FOURSQUARE_API AHandball : public AActor
{
    GENERATED_BODY()

public:
    AHandball();

    virtual void GetLifetimeReplicatedProps(
        TArray<FLifetimeProperty>& OutLifetimeProps
    ) const override;

    UHandballSimulationComponent* GetSimulationComponent() const
    {
        return SimulationComponent;
    }

    const FHandballState& GetBallState() const
    {
        return BallState;
    }

    void SetBallState(const FHandballState& NewState);

protected:
    virtual void BeginPlay() override;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Handball")
    TObjectPtr<USceneComponent> SceneRoot;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Handball")
    TObjectPtr<UStaticMeshComponent> BallMesh;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Handball")
    TObjectPtr<UHandballSimulationComponent> SimulationComponent;

    UPROPERTY(ReplicatedUsing = OnRep_BallState,
        VisibleInstanceOnly, BlueprintReadOnly, Category = "Handball")
    FHandballState BallState;

    UPROPERTY(Transient)
    TObjectPtr<UMaterialInstanceDynamic> SpeedMaterial;

    UFUNCTION()
    void OnRep_BallState();

    void ApplyBallState();
    void UpdateSpeedMaterial();
};
