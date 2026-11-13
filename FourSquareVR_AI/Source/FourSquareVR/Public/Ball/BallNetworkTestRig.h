#pragma once
#include "CoreMinimal.h"
#include "Ball/BallHandTestRig.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/PlayerController.h"
#include "BallNetworkTestRig.generated.h"
/** Opt-in multi-process diagnostic. Never selected by the project's GameMode or VR Pawn. */
UCLASS(NotBlueprintable)
class FOURSQUAREVR_API ABallNetworkTestPawn : public ABallHandTestRig
{
	GENERATED_BODY()
  public:
	ABallNetworkTestPawn();
	virtual void BeginPlay() override;
	virtual void Tick(float Delta) override;
};
UCLASS(NotBlueprintable)
class FOURSQUAREVR_API ABallNetworkTestController : public APlayerController
{
	GENERATED_BODY()
  public:
	virtual void Tick(float Delta) override;

  private:
	double Started = 0, NextReport = 0;
	bool bReset = false, bConfiguration = false, bClientGuardTest = false, bOwnershipProbe = false;
};
UCLASS(NotBlueprintable)
class FOURSQUAREVR_API ABallNetworkTestGameMode : public AGameModeBase
{
	GENERATED_BODY()
  public:
	ABallNetworkTestGameMode();
	virtual void BeginPlay() override;
	virtual void Tick(float Delta) override;

  private:
	double Started = 0;
};
