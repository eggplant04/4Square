#pragma once
#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "BallHandTestRig.generated.h"
class UMotionControllerComponent;
class UCameraComponent;
class UVRHandCollisionComponent;

/** Explicit no-headset diagnostic trajectories through the SAME capture/history/CCD pipeline. */
UCLASS(Blueprintable)
class FOURSQUAREVR_API ABallHandTestRig : public APawn
{
	GENERATED_BODY()
  public:
	ABallHandTestRig();
	virtual void Tick(float Delta) override;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ball|Test")
	TObjectPtr<USceneComponent> TrackingOrigin;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ball|Test")
	TObjectPtr<UCameraComponent> Head;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ball|Test")
	TObjectPtr<UMotionControllerComponent> Left;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ball|Test")
	TObjectPtr<UMotionControllerComponent> Right;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ball|Test")
	TObjectPtr<UVRHandCollisionComponent> Hands;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Test")
	bool bAnimate = true;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Test", meta = (ClampMin = "0"))
	double FrequencyHz = .5;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Test", meta = (ClampMin = "0"))
	double Amplitude = 50;
};
