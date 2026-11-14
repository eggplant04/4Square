
#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Character.h"
#include "TestPawn.generated.h"

class UStaticMeshComponent;
class UHandballHandComponent;
class USpringArmComponent;
class UCameraComponent;
class UInputComponent;

UCLASS()
class FOURSQUARE_API ATestPawn : public ACharacter
{
    GENERATED_BODY()

public:
    ATestPawn();

    virtual void OnConstruction(const FTransform& Transform) override;
    virtual void BeginPlay() override;
    virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

protected:
    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Handball")
    TObjectPtr<UStaticMeshComponent> HeadMesh;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Handball")
    TObjectPtr<UHandballHandComponent> HeadCollider;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera")
    TObjectPtr<USpringArmComponent> CameraBoom;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Camera")
    TObjectPtr<UCameraComponent> FollowCamera;

private:
    void MoveForward(float Value);
    void MoveRight(float Value);
    void MoveUp(float Value);
};
