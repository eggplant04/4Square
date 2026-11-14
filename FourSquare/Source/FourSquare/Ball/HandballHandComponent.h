#pragma once

#include "CoreMinimal.h"
#include "Components/BoxComponent.h"
#include "HandballHandComponent.generated.h"

UCLASS(ClassGroup = (Handball), meta = (BlueprintSpawnableComponent))
class FOURSQUARE_API UHandballHandComponent : public UBoxComponent
{
    GENERATED_BODY()

public:
    UHandballHandComponent();

    UFUNCTION(BlueprintCallable, Category = "Handball|Grab")
    bool TryGrabBall();

    UFUNCTION(BlueprintCallable, Category = "Handball|Grab")
    void ReleaseBall();
    
    bool HasValidTracking() const;

protected:
    virtual void BeginPlay() override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;
};