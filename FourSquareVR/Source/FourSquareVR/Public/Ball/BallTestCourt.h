#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "BallTestCourt.generated.h"

/** Native test fixture; creates ordinary static collision, without editing the existing VR map. */
UCLASS()
class FOURSQUAREVR_API ABallTestCourt : public AActor
{
	GENERATED_BODY()
public:
	ABallTestCourt();
	virtual void OnConstruction(const FTransform& Transform) override;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Ball|TestCourt") bool bIncludeCeiling = true;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category="Ball|TestCourt") TObjectPtr<UStaticMeshComponent> Ceiling;
};
