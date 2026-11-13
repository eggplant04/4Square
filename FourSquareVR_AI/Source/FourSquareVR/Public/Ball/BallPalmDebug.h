#pragma once
#include "CoreMinimal.h"
class UWorld;
struct FOURSQUAREVR_API FBallPalmDebug
{
	/** Wire curves lie on the same core-box + sphere surface used by collision. Debug only. */
	static void Draw(UWorld *World, const FTransform &Transform, const FVector &HalfExtents, double Radius,
					 FColor Color);
};
