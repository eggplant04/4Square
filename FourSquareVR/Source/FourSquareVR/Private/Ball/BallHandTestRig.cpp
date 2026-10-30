#include "Ball/BallHandTestRig.h"
#include "Ball/VRHandCollisionComponent.h"
#include "Camera/CameraComponent.h"
#include "MotionControllerComponent.h"
#include "Engine/World.h"

ABallHandTestRig::ABallHandTestRig()
{
	PrimaryActorTick.bCanEverTick = true;
	PrimaryActorTick.TickGroup = TG_PrePhysics;
	AutoPossessPlayer = EAutoReceiveInput::Disabled;
	TrackingOrigin = CreateDefaultSubobject<USceneComponent>(TEXT("TrackingOrigin"));
	SetRootComponent(TrackingOrigin);
	Head = CreateDefaultSubobject<UCameraComponent>(TEXT("Head"));
	Head->SetupAttachment(TrackingOrigin);
	Head->SetRelativeLocation(FVector(0, 0, 150));
	Head->bLockToHmd = false;
	Left = CreateDefaultSubobject<UMotionControllerComponent>(TEXT("MotionControllerLeftGrip"));
	Left->SetupAttachment(TrackingOrigin);
	Left->SetTrackingMotionSource(TEXT("LeftGrip"));
	Left->SetRelativeLocation(FVector(-40, 0, 100));
	Right = CreateDefaultSubobject<UMotionControllerComponent>(TEXT("MotionControllerRightGrip"));
	Right->SetupAttachment(TrackingOrigin);
	Right->SetTrackingMotionSource(TEXT("RightGrip"));
	Right->SetRelativeLocation(FVector(0, 35, 100));
	Hands = CreateDefaultSubobject<UVRHandCollisionComponent>(TEXT("BallHandCollision"));
	Hands->bAllowScriptedTrackingForTests = true;
	Hands->bDebugDraw = true;
}
void ABallHandTestRig::Tick(float Delta)
{
	Super::Tick(Delta);
	if (bAnimate)
	{
		Left->SetRelativeLocation(
			FVector(-40 + Amplitude * FMath::Sin(GetWorld()->GetTimeSeconds() * 2 * PI * FrequencyHz), 0, 100));
	}
}
