#include "Ball/BallPoseAdmission.h"
#include "Ball/BallPalmGeometry.h"
#include "HAL/PlatformTime.h"
namespace
{
bool SameObservation(const FBallHandObservation &A, const FBallHandObservation &B)
{
	return A.CaptureSeconds == B.CaptureSeconds && A.Sequence == B.Sequence && A.bTracked == B.bTracked &&
		   A.Side == B.Side && A.ControllerPosition == B.ControllerPosition &&
		   A.ControllerOrientation == B.ControllerOrientation && A.HeadPosition == B.HeadPosition &&
		   A.HeadOrientation == B.HeadOrientation && A.OriginPosition == B.OriginPosition &&
		   A.OriginOrientation == B.OriginOrientation && A.ConfigurationVersion == B.ConfigurationVersion &&
		   A.OriginRevision == B.OriginRevision;
}
bool Derive(FBallHandSample &S, const FBallHandSample *Previous, const FBallHandTrackingSettings &T)
{
	S.LinearVelocity = S.AngularVelocity = S.ExtrapolationVelocity = FVector::ZeroVector;
	S.bCollisionReady = false;
	if (!S.Pose.bTracked)
	{
		S.Rejection = EBallTrackingRejection::Untracked;
		return true;
	}
	if (!Previous || Previous->Generation != S.Generation || !Previous->Pose.bTracked)
	{
		S.Rejection = EBallTrackingRejection::WarmingUp;
		return true;
	}
	const double Dt = S.Pose.TimelineSeconds - Previous->Pose.TimelineSeconds;
	if (Dt < 1.e-6 || Dt > T.MaximumSampleGap)
	{
		S.Rejection = EBallTrackingRejection::WarmingUp;
		return true;
	}
	const auto &P = S.Pose, &B = Previous->Pose;
	S.LinearVelocity = (P.ControllerPosition - B.ControllerPosition) / Dt;
	const FVector Angle = FBallPalmGeometry::RotationVector(B.ControllerOrientation, P.ControllerOrientation);
	S.AngularVelocity = Angle / Dt;
	if (S.LinearVelocity.Size() > T.MaximumSpeed || Angle.Size() > T.MaximumSampleRotation ||
		S.AngularVelocity.Size() > T.MaximumAngularSpeed ||
		(P.HeadPosition - B.HeadPosition).Size() / Dt > T.MaximumHeadSpeed ||
		(Previous->bCollisionReady &&
		 (S.LinearVelocity - Previous->LinearVelocity).Size() / Dt > T.MaximumAcceleration))
		return false;
	S.ExtrapolationVelocity = S.LinearVelocity;
	if (Previous->bCollisionReady && T.ExtrapolationFilterSeconds > 0 && T.MaximumExtrapolation > 0)
	{
		const FVector Smoothed = FMath::Lerp(Previous->ExtrapolationVelocity, S.LinearVelocity,
											 1 - FMath::Exp(-Dt / T.ExtrapolationFilterSeconds));
		S.ExtrapolationVelocity += (Smoothed - S.LinearVelocity)
									   .GetClampedToMaxSize(T.MaximumExtrapolationFilterError / T.MaximumExtrapolation);
	}
	S.Rejection = EBallTrackingRejection::None;
	S.bCollisionReady = true;
	return true;
}
} // namespace
EBallInputDisposition FBallNetworkPoseHistory::Insert(const FBallHandObservation &P, uint32 Generation,
													  const FBallHandTrackingSettings &T)
{
	if (!P.IsValid() || !Generation || !T.IsValid() || Samples.Num() != BallNet::PoseHistory)
		return EBallInputDisposition::Malformed;
	for (int32 I = 0; I < Count; ++I)
		if (Samples[I].Pose.Sequence == P.Sequence)
			return SameObservation(Samples[I].Pose, P) ? EBallInputDisposition::Duplicate
													   : EBallInputDisposition::ConflictingDuplicate;
	int32 Index = 0;
	while (Index < Count && Samples[Index].Pose.TimelineSeconds < P.TimelineSeconds)
		++Index;
	if ((Index < Count && Samples[Index].Pose.TimelineSeconds == P.TimelineSeconds) ||
		(Index > 0 && Generation < Samples[Index - 1].Generation) ||
		(Index < Count && Generation > Samples[Index].Generation) ||
		(Index > 0 && !FBallFinalityPolicy::IsSequenceNewer(P.Sequence, Samples[Index - 1].Pose.Sequence)) ||
		(Index < Count && !FBallFinalityPolicy::IsSequenceNewer(Samples[Index].Pose.Sequence, P.Sequence)))
		return EBallInputDisposition::OutOfOrder;
	if (Count == Samples.Num() && (Index == 0 || Samples[0].Pose.TimelineSeconds >= PinnedFrom))
		return EBallInputDisposition::Capacity;
	// Bounded transactional scratch. No partially accepted trajectory/filter changes on failure.
	FBallHandSample Candidate[BallNet::PoseHistory];
	int32 N = Count, Insert = Index, Begin = 0;
	if (N == Samples.Num())
	{
		Begin = 1;
		--N;
		--Insert;
	}
	for (int32 I = 0; I < N; ++I)
		Candidate[I] = Samples[Begin + I];
	for (int32 I = N; I > Insert; --I)
		Candidate[I] = Candidate[I - 1];
	Candidate[Insert] = {};
	Candidate[Insert].Pose = P;
	Candidate[Insert].Generation = Generation;
	++N;
	for (int32 I = Insert; I < N; ++I)
	{
		if (!Derive(Candidate[I], I ? &Candidate[I - 1] : nullptr, T))
			return EBallInputDisposition::InvalidMotion;
	}
	for (int32 I = 0; I < N; ++I)
		Samples[I] = Candidate[I];
	Count = N;
	return Index < Count - 1 ? EBallInputDisposition::LateHistorical : EBallInputDisposition::Accepted;
}
void FBallPoseAdmission::Initialize(const FBallPeerProfile &P)
{
	Profile = P;
	for (auto &H : Hands)
		H.Initialize();
	Statistics = {};
	Rate = {};
	LastPacketSequence = 0;
	for (int32 Side = 0; Side < 2; ++Side)
	{
		LastCaptureGeneration[Side] = 0;
		MotionGeneration[Side] = 1;
		GenerationHead[Side] = 0;
		SampleRates[Side] = {};
		SampleRates[Side].Tokens = 16;
		for (auto &G : Generations[Side])
			G = {};
	}
	EarliestDirtyTimeline = TNumericLimits<double>::Max();
}
void FBallPoseAdmission::ChangeProfile(const FBallPeerProfile &P)
{
	if (P.Epoch != Profile.Epoch || P.Timebase != Profile.Timebase)
	{
		Initialize(P);
		return;
	}
	Profile = P;
	for (int32 Side = 0; Side < 2; ++Side)
	{
		LastCaptureGeneration[Side] = 0;
		++MotionGeneration[Side];
		for (auto &G : Generations[Side])
			G = {};
	}
}
EBallInputDisposition FBallPoseAdmission::Admit(const FBallPoseBatch &B, const FBallClockEstimate &Clock, bool Ready,
												double Now, double Timeline, double SimulatedThrough)
{
	const double Begin = FPlatformTime::Seconds();
	++Statistics.Packets;
	const auto Finish = [&](EBallInputDisposition R) {
		Statistics.AdmissionMicroseconds += (FPlatformTime::Seconds() - Begin) * 1.e6;
		if (R != EBallInputDisposition::Accepted && R != EBallInputDisposition::Duplicate &&
			R != EBallInputDisposition::LateHistorical)
			++Statistics.Rejected;
		return R;
	};
	if (!Rate.Consume(Now, Profile.Transport.MaximumRPCsPerSecond))
		return Finish(EBallInputDisposition::RateLimited);
	if (!Profile.IsValid() || B.Count < 1 || B.Count > BallNet::MaximumPoseSamples || !B.Sequence ||
		!B.CaptureGeneration)
		return Finish(EBallInputDisposition::Malformed);
	if (B.Epoch != Profile.Epoch || B.Timebase != Profile.Timebase || B.ProfileVersion != Profile.Version ||
		B.OriginRevision != Profile.OriginRevision)
		return Finish(EBallInputDisposition::WrongContext);
	if (!Ready || !Clock.IsValid() || Clock.Uncertainty > Profile.Transport.MaximumClockUncertainty)
		return Finish(EBallInputDisposition::ClockUncertain);
	if (!FMath::IsFinite(B.SentSeconds))
		return Finish(EBallInputDisposition::Malformed);
	if (LastPacketSequence && FBallFinalityPolicy::IsSequenceNewer(B.Sequence, LastPacketSequence))
		Statistics.SequenceGaps += FMath::Min(uint32(B.Sequence - LastPacketSequence - 1), uint32(10000));
	if (!LastPacketSequence || FBallFinalityPolicy::IsSequenceNewer(B.Sequence, LastPacketSequence))
		LastPacketSequence = B.Sequence;
	EBallInputDisposition Overall = EBallInputDisposition::Duplicate;
	for (int32 I = 0; I < B.Count; ++I)
	{
		auto P = B.Samples[I];
		if (uint8(P.Side) > 1 || P.CaptureSeconds > B.SentSeconds + .001)
			return Finish(EBallInputDisposition::Malformed);
		const uint8 Side = uint8(P.Side);
		const uint32 ClientGeneration = B.Generations[I] ? B.Generations[I] : B.CaptureGeneration;
		uint32 Generation = 0;
		for (const auto &G : Generations[Side])
			if (G.Client == ClientGeneration)
				Generation = G.Server;
		const bool NewGeneration = !Generation;
		if (!Generation)
		{
			if (LastCaptureGeneration[Side] &&
				(!FBallFinalityPolicy::IsSequenceNewer(ClientGeneration, LastCaptureGeneration[Side]) ||
				 uint32(ClientGeneration - LastCaptureGeneration[Side]) > 16 || MotionGeneration[Side] == MAX_uint32))
				return Finish(EBallInputDisposition::WrongContext);
			Generation = MotionGeneration[Side] + (LastCaptureGeneration[Side] ? 1 : 0);
		}
		P.ParticipantId = Profile.Participant;
		P.bTracked = P.bTracked && (Side == 0 ? Profile.bEnableLeft : Profile.bEnableRight);
		P.ConfigurationVersion = Profile.Version;
		P.Timebase = Profile.Timebase;
		P.OriginRevision = Profile.OriginRevision;
		const double Capture = Clock.RemoteToLocal(P.CaptureSeconds);
		if (!FMath::IsFinite(Capture) || !P.IsValid())
			return Finish(EBallInputDisposition::Malformed);
		if (Now - Capture > Profile.Transport.MaximumPoseAge)
			return Finish(EBallInputDisposition::TooOld);
		if (Capture - Now > Profile.Transport.MaximumFuturePose)
			return Finish(EBallInputDisposition::Future);
		if (P.WorldToMeters != Profile.WorldToMeters ||
			!P.OriginPosition.Equals(Profile.Origin, Profile.Transport.OriginPositionTolerance) ||
			P.OriginOrientation.AngularDistance(Profile.OriginRotation) > Profile.Transport.OriginAngularTolerance ||
			(P.ControllerPosition - P.HeadPosition).Size() > Profile.Tracking.MaximumReach ||
			(P.HeadPosition - Profile.Origin).Size() > Profile.Tracking.MaximumReach * 2)
			return Finish(EBallInputDisposition::InvalidOrigin);
		P.TimelineSeconds =
			Timeline - (Now - Capture); // frozen admission mapping; never clamped to current simulation time.
		if (P.TimelineSeconds < Profile.EffectiveTimelineSeconds)
			return Finish(EBallInputDisposition::WrongContext);
		auto &H = Hands[uint8(P.Side)];
		bool Known = false;
		for (const auto &S : H.View())
			if (S.Pose.Sequence == P.Sequence)
			{
				Known = true;
				break;
			}
		if (!Known && !SampleRates[Side].Consume(Now, Profile.Transport.MaximumSamplesPerSecond, 1, 16))
			return Finish(EBallInputDisposition::RateLimited);
		double DirtyStart = P.TimelineSeconds;
		for (const auto &S : H.View())
			if (S.Pose.TimelineSeconds < P.TimelineSeconds)
				DirtyStart = S.Pose.TimelineSeconds;
		const auto R = H.Insert(P, Generation, Profile.Tracking);
		if (R == EBallInputDisposition::Duplicate)
			++Statistics.Duplicates;
		else if (R == EBallInputDisposition::Accepted || R == EBallInputDisposition::LateHistorical)
		{
			if (NewGeneration)
			{
				LastCaptureGeneration[Side] = ClientGeneration;
				MotionGeneration[Side] = Generation;
				Generations[Side][GenerationHead[Side]] = {ClientGeneration, Generation};
				GenerationHead[Side] = (GenerationHead[Side] + 1) % 16;
			}
			++Statistics.Accepted;
			Overall = EBallInputDisposition::Accepted;
			if (R == EBallInputDisposition::LateHistorical || DirtyStart < SimulatedThrough)
				++Statistics.Late;
			EarliestDirtyTimeline =
				FMath::Min(EarliestDirtyTimeline, DirtyStart); // Stage7 consumes this; no resimulation in Stage5.
		}
		else
			return Finish(R);
	}
	return Finish(Overall);
}
bool FBallPoseAdmission::BuildMotion(double Start, FBallMotionFrame &F, EBallTrackingRejection *LeftReason,
									 EBallTrackingRejection *RightReason) const
{
	EBallTrackingRejection L, R;
	const bool Left =
		FBallHandPoseHistory::BuildAcceptedMotion(Hands[0].View(), Start, Profile.Left, Profile.Tracking,
												  Profile.Participant, EBallHandSide::Left, F, L, Profile.Version);
	const bool Right =
		FBallHandPoseHistory::BuildAcceptedMotion(Hands[1].View(), Start, Profile.Right, Profile.Tracking,
												  Profile.Participant, EBallHandSide::Right, F, R, Profile.Version);
	if (LeftReason)
		*LeftReason = L;
	if (RightReason)
		*RightReason = R;
	return Left && Right;
}
