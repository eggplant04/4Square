#pragma once

#include "CoreMinimal.h"
#include "HandballTypes.generated.h"

USTRUCT(BlueprintType)
struct FHandballState
{
    GENERATED_BODY()

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector Position = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite)
    FVector Velocity = FVector::ZeroVector;
};