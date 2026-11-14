
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "TestingGameMode.generated.h"

UCLASS()
class FOURSQUARE_API ATestingGameMode : public AGameModeBase
{
    GENERATED_BODY()

public:
    virtual UClass* GetDefaultPawnClassForController_Implementation(
        AController* InController) override;

protected:
    UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Handball|Players")
    TSubclassOf<APawn> VRPawnClass;

    UPROPERTY(EditDefaultsOnly, BlueprintReadOnly, Category = "Handball|Players")
    TSubclassOf<APawn> TestPawnClass;
};
