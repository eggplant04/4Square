#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "HandballTypes.h"
#include "Handball.generated.h"

class USceneComponent;
class UStaticMeshComponent; 
class UHandballSimulationComponent;

UCLASS()
class FOURSQUARE_API AHandball : public AActor
{
    GENERATED_BODY()

public:
    AHandball();
    
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
    
    UPROPERTY(VisibleInstanceOnly, BlueprintReadOnly, Category = "Handball")
    FHandballState BallState;
};