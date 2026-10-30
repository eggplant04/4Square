#pragma once
#include "CoreMinimal.h"
#include "UObject/Interface.h"
#include "Ball/BallMotionHistory.h"
#include "BallRewindableCollider.generated.h"

UINTERFACE(MinimalAPI, meta = (CannotImplementInterfaceInBlueprint))
class UBallRewindableCollider : public UInterface
{
	GENERATED_BODY()
};
/** Recording boundary only: replay receives the returned numeric frame, never this live object. */
class FOURSQUAREVR_API IBallRewindableCollider
{
	GENERATED_BODY()
  public:
	virtual bool AppendRecordedMotion(FBallMotionFrame &Frame, double StartSeconds) = 0;
};
