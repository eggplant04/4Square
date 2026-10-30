#include "Ball/BallHandTypes.h"
bool FBallPalmSettings::IsValid() const
{
	return FMath::IsFinite(Radius) && Radius > 0 && !CenterOffset.ContainsNaN() && Material.IsValid() &&
		   FMath::IsFinite(MaximumCurveError) && MaximumCurveError > 0;
}
bool FBallHandTrackingSettings::IsValid() const
{
	return FMath::IsFinite(MaximumSampleGap) && MaximumSampleGap > 0 && FMath::IsFinite(MaximumExtrapolation) &&
		   MaximumExtrapolation >= 0 && MaximumExtrapolation <= MaximumSampleGap &&
		   FMath::IsFinite(ExtrapolationFilterSeconds) && ExtrapolationFilterSeconds >= 0 &&
		   FMath::IsFinite(MaximumExtrapolationFilterError) && MaximumExtrapolationFilterError >= 0 &&
		   FMath::IsFinite(MaximumSpeed) && MaximumSpeed > 0 && FMath::IsFinite(MaximumAngularSpeed) &&
		   MaximumAngularSpeed > 0 && FMath::IsFinite(MaximumAcceleration) && MaximumAcceleration > 0 &&
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
