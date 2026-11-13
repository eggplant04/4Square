#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "HandballBarrier.generated.h"

class UStaticMeshComponent;

UCLASS()
class FOURSQUARE_API AHandballBarrier : public AActor
{
    GENERATED_BODY()

public:
    AHandballBarrier();

protected:
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Handball")
    TObjectPtr<UStaticMeshComponent> BarrierMesh;
};