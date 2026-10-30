#include "Ball/BallHandPoseHistory.h"

namespace
{
FVector AngularVelocity(const FQuat &A, const FQuat &B, double Dt)
{
	FQuat D = B * A.Inverse();
	D.Normalize();
	if (D.W < 0)
	{
		D = D * -1;
	}
	FVector Axis;
	double Angle;
	D.ToAxisAndAngle(Axis, Angle);
	return Axis.GetSafeNormal() * Angle / Dt;
}
FVector PalmCenter(const FBallHandObservation &A, const FBallHandObservation &B, double Alpha, const FVector &Offset)
{
	return FMath::Lerp(A.ControllerPosition, B.ControllerPosition, Alpha) +
		   FQuat::Slerp(A.ControllerOrientation, B.ControllerOrientation, Alpha).RotateVector(Offset);
}
} // namespace
bool FBallHandPoseHistory::Initialize(int32 Capacity)
{
	if (Capacity < 3 || Capacity > 4096)
	{
		return false;
	}
	Samples.SetNum(Capacity);
	Head = Count = 0;
	Generation = 1;
	bNeedsWarmup = true;
	return true;
}
const FBallHandSample &FBallHandPoseHistory::At(int32 Index) const
{
	return Samples[(Head + Index) % Samples.Num()];
}
const FBallHandSample *FBallHandPoseHistory::GetLatest() const
{
	return Count ? &At(Count - 1) : nullptr;
}
void FBallHandPoseHistory::Push(const FBallHandSample &S)
{
	if (Count == Samples.Num())
	{
		Head = (Head + 1) % Samples.Num();
		--Count;
	}
	Samples[(Head + Count) % Samples.Num()] = S;
	++Count;
}
void FBallHandPoseHistory::BreakContinuity()
{
	if (!bNeedsWarmup && Generation < MAX_uint32)
	{
		++Generation;
	}
	bNeedsWarmup = true;
}
void FBallHandPoseHistory::NotifyDiscontinuity()
{
	BreakContinuity();
}
EBallTrackingRejection FBallHandPoseHistory::Add(const FBallHandObservation &P,
												 const FBallHandTrackingSettings &Settings)
{
	if (!Settings.IsValid() || Samples.Num() < 3 || !P.IsValid() || Generation == MAX_uint32)
	{
		BreakContinuity();
		return EBallTrackingRejection::InvalidPose;
	}
	const auto *Last = GetLatest();
	if (Last && (P.CaptureSeconds <= Last->Pose.CaptureSeconds ||
				 !FBallFinalityPolicy::IsSequenceNewer(P.Sequence, Last->Pose.Sequence) ||
				 (P.Timebase == Last->Pose.Timebase && P.TimelineSeconds <= Last->Pose.TimelineSeconds)))
	{
		return EBallTrackingRejection::OutOfOrder;
	}
	FBallHandSample S;
	S.Pose = P;
	S.Generation = Generation;
	auto Reject = [&](EBallTrackingRejection Reason) {
		BreakContinuity();
		S.Generation = Generation;
		S.Rejection = Reason;
		Push(S);
		return Reason;
	};
	if (!P.bTracked)
	{
		return Reject(EBallTrackingRejection::Untracked);
	}
	if ((P.ControllerPosition - P.HeadPosition).Size() > Settings.MaximumReach)
	{
		return Reject(EBallTrackingRejection::ExcessiveReach);
	}
	if (Last &&
		(P.Timebase != Last->Pose.Timebase || P.OriginRevision != Last->Pose.OriginRevision ||
		 P.ParticipantId != Last->Pose.ParticipantId || P.Side != Last->Pose.Side ||
		 P.ConfigurationVersion != Last->Pose.ConfigurationVersion || P.WorldToMeters != Last->Pose.WorldToMeters ||
		 !P.OriginPosition.Equals(Last->Pose.OriginPosition, 0.001) ||
		 !P.OriginOrientation.Equals(Last->Pose.OriginOrientation, 1.e-6)))
	{
		BreakContinuity();
	}
	if (bNeedsWarmup)
	{
		S.Generation = Generation;
		S.Rejection = EBallTrackingRejection::WarmingUp;
		bNeedsWarmup = false;
		Push(S);
		return S.Rejection;
	}
	const double Dt = P.TimelineSeconds - Last->Pose.TimelineSeconds;
	if (Dt < 1.e-6 || Dt > Settings.MaximumSampleGap)
	{
		return Reject(EBallTrackingRejection::Stale);
	}
	S.LinearVelocity = (P.ControllerPosition - Last->Pose.ControllerPosition) / Dt;
	S.AngularVelocity = AngularVelocity(Last->Pose.ControllerOrientation, P.ControllerOrientation, Dt);
	if (S.LinearVelocity.Size() > Settings.MaximumSpeed || S.AngularVelocity.Size() > Settings.MaximumAngularSpeed ||
		(P.HeadPosition - Last->Pose.HeadPosition).Size() / Dt > Settings.MaximumHeadSpeed)
	{
		return Reject(EBallTrackingRejection::ExcessiveVelocity);
	}
	if (Last->bCollisionReady && (S.LinearVelocity - Last->LinearVelocity).Size() / Dt > Settings.MaximumAcceleration)
	{
		return Reject(EBallTrackingRejection::ExcessiveAcceleration);
	}
	S.ExtrapolationVelocity = S.LinearVelocity;
	if (Last->bCollisionReady && Settings.ExtrapolationFilterSeconds > 0 && Settings.MaximumExtrapolation > 0)
	{
		const double Alpha = 1 - FMath::Exp(-Dt / Settings.ExtrapolationFilterSeconds);
		const FVector Smoothed = FMath::Lerp(Last->ExtrapolationVelocity, S.LinearVelocity, Alpha);
		S.ExtrapolationVelocity = S.LinearVelocity + (Smoothed - S.LinearVelocity)
														 .GetClampedToMaxSize(Settings.MaximumExtrapolationFilterError /
																			  Settings.MaximumExtrapolation);
	}
	S.Generation = Generation;
	S.bCollisionReady = true;
	S.Rejection = EBallTrackingRejection::None;
	Push(S);
	return S.Rejection;
}
bool FBallHandPoseHistory::BuildMotion(double StartSeconds, const FBallPalmSettings &Palm,
									   const FBallHandTrackingSettings &Settings, uint32 Participant,
									   EBallHandSide Side, FBallMotionFrame &Frame, EBallTrackingRejection &Reason,
									   uint32 ConfigurationVersion) const
{
	Reason = EBallTrackingRejection::Stale;
	if (!FMath::IsFinite(StartSeconds) || !Palm.IsValid() || !Settings.IsValid() || !Participant || !Frame.IsValid())
	{
		Reason = EBallTrackingRejection::InvalidPose;
		return false;
	}
	const double EndSeconds = StartSeconds + Frame.DeltaTime;
	FBallMotionSegment Segments[BallMotionLimits::SegmentsPerCollider];
	int32 N = 0;
	uint32 SelectedGeneration = 0, Origin = 0;
	FQuat SelectedOrientation = FQuat::Identity;
	double SelectedWorldToMeters = 100;
	auto Append = [&](const FBallHandSample &A, const FBallHandSample &B, double Begin, double End, bool bExtrapolate) {
		if (End <= Begin + 1.e-12)
		{
			return true;
		}
		if (SelectedGeneration && SelectedGeneration != B.Generation)
		{
			N = 0;
		}
		SelectedGeneration = B.Generation;
		Origin = B.Pose.OriginRevision;
		SelectedOrientation = B.Pose.ControllerOrientation;
		SelectedWorldToMeters = B.Pose.WorldToMeters;
		const double SampleDt = B.Pose.TimelineSeconds - A.Pose.TimelineSeconds;
		FBallHandObservation P = A.Pose, Q = B.Pose;
		if (bExtrapolate)
		{
			P = B.Pose;
			Q = P;
			Q.ControllerPosition += B.ExtrapolationVelocity * (End - P.TimelineSeconds);
			const double Angle = B.AngularVelocity.Size() * (End - P.TimelineSeconds);
			if (Angle >= PI - 1.e-6)
			{
				Reason = EBallTrackingRejection::ExcessiveVelocity;
				return false;
			}
			if (Angle > 1.e-12)
			{
				Q.ControllerOrientation = FQuat(B.AngularVelocity.GetSafeNormal(), Angle) * P.ControllerOrientation;
				Q.ControllerOrientation.Normalize();
			}
			Q.TimelineSeconds = End;
		}
		const double Span = Q.TimelineSeconds - P.TimelineSeconds;
		const double Alpha0 = (Begin - P.TimelineSeconds) / Span, Alpha1 = (End - P.TimelineSeconds) / Span;
		const double Angle = P.ControllerOrientation.AngularDistance(Q.ControllerOrientation) * (Alpha1 - Alpha0);
		int32 Divisions = 1;
		while (Palm.CenterOffset.Size() * (1 - FMath::Cos(Angle / (2 * Divisions))) > Palm.MaximumCurveError &&
			   Divisions < BallMotionLimits::SegmentsPerCollider)
		{
			Divisions *= 2;
		}
		if (Palm.CenterOffset.Size() * (1 - FMath::Cos(Angle / (2 * Divisions))) > Palm.MaximumCurveError ||
			N + Divisions > BallMotionLimits::SegmentsPerCollider)
		{
			Reason = EBallTrackingRejection::Capacity;
			return false;
		}
		for (int32 J = 0; J < Divisions; ++J)
		{
			const double T0 = FMath::Lerp(Begin, End, double(J) / Divisions),
						 T1 = FMath::Lerp(Begin, End, double(J + 1) / Divisions);
			auto &S = Segments[N++];
			S.StartFraction = FMath::Clamp((T0 - StartSeconds) / Frame.DeltaTime, 0.0, 1.0);
			S.EndFraction = FMath::Clamp((T1 - StartSeconds) / Frame.DeltaTime, 0.0, 1.0);
			S.Start = PalmCenter(P, Q, (T0 - P.TimelineSeconds) / Span, Palm.CenterOffset);
			S.End = PalmCenter(P, Q, (T1 - P.TimelineSeconds) / Span, Palm.CenterOffset);
			if ((S.End - S.Start).Size() / (T1 - T0) > Settings.MaximumSpeed)
			{
				Reason = EBallTrackingRejection::ExcessiveVelocity;
				return false;
			}
			S.SurfaceAngularVelocity = Palm.bRotatingSurfaceFriction ? B.AngularVelocity : FVector::ZeroVector;
			S.FirstSample = A.Pose.Sequence;
			S.SecondSample = B.Pose.Sequence;
		}
		return SampleDt > 0;
	};
	for (int32 I = 1; I < Count; ++I)
	{
		const auto &A = At(I - 1);
		const auto &B = At(I);
		if (!A.bCollisionReady || !B.bCollisionReady || A.Generation != B.Generation ||
			A.Pose.Timebase != Frame.Timebase || B.Pose.Timebase != Frame.Timebase ||
			B.Pose.ParticipantId != Participant || B.Pose.Side != Side ||
			B.Pose.ConfigurationVersion != ConfigurationVersion)
		{
			continue;
		}
		if (!Append(A, B, FMath::Max(StartSeconds, A.Pose.TimelineSeconds),
					FMath::Min(EndSeconds, B.Pose.TimelineSeconds), false))
		{
			return false;
		}
	}
	if (Count > 1)
	{
		const auto &B = At(Count - 1);
		const auto &A = At(Count - 2);
		if (!bNeedsWarmup && B.Generation == Generation && B.bCollisionReady && A.Generation == B.Generation &&
			B.Pose.Timebase == Frame.Timebase && B.Pose.ParticipantId == Participant && B.Pose.Side == Side &&
			B.Pose.ConfigurationVersion == ConfigurationVersion && EndSeconds > B.Pose.TimelineSeconds)
		{
			const double ExtrapolatedEnd =
				FMath::Min(EndSeconds, B.Pose.TimelineSeconds + Settings.MaximumExtrapolation);
			if (!Append(A, B, FMath::Max(StartSeconds, B.Pose.TimelineSeconds), ExtrapolatedEnd, true))
			{
				return false;
			}
		}
	}
	if (!N)
	{
		return true;
	}
	FBallColliderMotion C;
	C.Id = BallMotionLimits::HandId(Participant, Side, SelectedGeneration);
	C.Participant = Participant;
	C.Side = Side;
	C.OriginRevision = Origin;
	C.Radius = Palm.Radius;
	C.Material = Palm.Material;
	C.Material.bOverride = true;
	C.Orientation = SelectedOrientation;
	C.ConfigurationVersion = ConfigurationVersion;
	C.WorldToMeters = SelectedWorldToMeters;
	if (!Frame.Add(C, MakeArrayView(Segments, N)))
	{
		Reason = EBallTrackingRejection::Capacity;
		return false;
	}
	Reason = EBallTrackingRejection::None;
	return true;
}
