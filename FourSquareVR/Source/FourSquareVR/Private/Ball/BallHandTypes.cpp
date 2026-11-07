#include "Ball/BallHandTypes.h"
bool FBallPalmSettings::IsValid() const
{
	return uint8(Shape) <= uint8(EBallPalmShape::RoundedBox) && !BoxDimensions.ContainsNaN() &&
		   BoxDimensions.GetMin() > 0 && FMath::IsFinite(RoundRadius) && RoundRadius >= 0 &&
		   RoundRadius <= .5 * BoxDimensions.GetMin() && FMath::IsFinite(CCDDistanceTolerance) &&
		   CCDDistanceTolerance >= 1.e-8 && CCDDistanceTolerance <= .001 && MaximumCCDIterations >= 1 &&
		   MaximumCCDIterations <= 2048 && !RotationOffset.ContainsNaN() && FaceMaterials.Front.IsValid() &&
		   FaceMaterials.Back.IsValid() && FaceMaterials.Side.IsValid() && FMath::IsFinite(Radius) && Radius > 0 &&
		   !CenterOffset.ContainsNaN() && Material.IsValid() && FMath::IsFinite(MaximumCurveError) &&
		   MaximumCurveError > 0;
}
bool FBallHandTrackingSettings::IsValid() const
{
	return FMath::IsFinite(MaximumSampleGap) && MaximumSampleGap > 0 && FMath::IsFinite(MaximumExtrapolation) &&
		   MaximumExtrapolation >= 0 && MaximumExtrapolation <= MaximumSampleGap &&
		   FMath::IsFinite(ExtrapolationFilterSeconds) && ExtrapolationFilterSeconds >= 0 &&
		   FMath::IsFinite(MaximumExtrapolationFilterError) && MaximumExtrapolationFilterError >= 0 &&
		   FMath::IsFinite(MaximumSpeed) && MaximumSpeed > 0 && FMath::IsFinite(MaximumAngularSpeed) &&
		   MaximumAngularSpeed > 0 && FMath::IsFinite(MaximumSampleRotation) && MaximumSampleRotation > 0 &&
		   MaximumSampleRotation < PI - 1.e-6 && FMath::IsFinite(MaximumAcceleration) && MaximumAcceleration > 0 &&
		   FMath::IsFinite(MaximumHeadSpeed) && MaximumHeadSpeed > 0 && FMath::IsFinite(MaximumReach) &&
		   MaximumReach > 0;
}
bool FBallHandObservation::IsValid() const
{
	return FMath::IsFinite(CaptureSeconds) && FMath::IsFinite(TimelineSeconds) && Timebase && OriginRevision &&
		   Sequence && ParticipantId && ConfigurationVersion && uint8(Side) <= uint8(EBallHandSide::Right) &&
		   !ControllerPosition.ContainsNaN() && !ControllerOrientation.ContainsNaN() &&
		   ControllerOrientation.IsNormalized() && !HeadPosition.ContainsNaN() && !HeadOrientation.ContainsNaN() &&
		   HeadOrientation.IsNormalized() && !OriginPosition.ContainsNaN() && !OriginOrientation.ContainsNaN() &&
		   OriginOrientation.IsNormalized() && FMath::IsFinite(WorldToMeters) && WorldToMeters > 0;
}
