#include "Ball/BallNetworkTypes.h"
#include "Ball/BallStateCodec.h"
#include "Serialization/MemoryReader.h"
#include "Serialization/MemoryWriter.h"

bool FBallWirePacket::NetSerialize(FArchive &Ar, UPackageMap *, bool &Success)
{
	uint32 Count = Bytes.Num();
	if (Ar.IsSaving() && Count > BallNet::MaximumPacketBytes)
	{
		Ar.SetError();
		Success = false;
		return true;
	}
	Ar.SerializeInt(Count, BallNet::MaximumPacketBytes + 1);
	if (Ar.IsError() || Count > BallNet::MaximumPacketBytes)
	{
		Ar.SetError();
		Success = false;
		return true;
	}
	if (Ar.IsLoading())
		Bytes.SetNum(Count);
	if (Count)
		Ar.Serialize(Bytes.GetData(), Count);
	Success = !Ar.IsError();
	return true;
}
bool FBallTransportSettings::IsValid() const
{
	const double Values[] = {MaximumSamplesPerSecond,
							 PoseRate,
							 MinimumPoseRate,
							 SnapshotRate,
							 RemotePoseRate,
							 MaximumRPCsPerSecond,
							 MaximumPoseAge,
							 MaximumFuturePose,
							 MaximumClockUncertainty,
							 MaximumClockDriftPPM,
							 OriginPositionTolerance,
							 OriginAngularTolerance,
							 VisualDelay};
	for (double V : Values)
		if (!FMath::IsFinite(V) || V < 0)
			return false;
	return MinimumPoseRate >= 10 && PoseRate >= MinimumPoseRate && PoseRate <= 120 && SnapshotRate >= 1 &&
		   SnapshotRate <= 60 && RemotePoseRate >= 1 && RemotePoseRate <= 60 && MaximumRPCsPerSecond >= PoseRate &&
		   MaximumRPCsPerSecond <= 240 && MaximumPoseAge > 0 && MaximumPoseAge <= 2 && MaximumFuturePose <= .1 &&
		   MaximumClockUncertainty <= .25 && MaximumClockDriftPPM <= 10000 && OriginPositionTolerance <= 5 &&
		   OriginAngularTolerance <= .1 && VisualDelay <= .1 && MaximumSamplesPerSecond >= 120 &&
		   MaximumSamplesPerSecond <= 480 && MaximumSamplesPerSecond * MaximumPoseAge + 16 <= BallNet::PoseHistory;
}
bool FBallPeerProfile::IsValid() const
{
	return Epoch.IsValid() && Participant && Version && Timebase && OriginRevision &&
		   FMath::IsFinite(EffectiveTimelineSeconds) && Left.IsValid() && Right.IsValid() && Tracking.IsValid() &&
		   Transport.IsValid() && !Origin.ContainsNaN() && !OriginRotation.ContainsNaN() &&
		   OriginRotation.IsNormalized() && FMath::IsFinite(WorldToMeters) && WorldToMeters > 0;
}
bool FBallNetworkSnapshot::IsValid() const
{
	return State.IsValid() && Physics.IsValid() && History.IsValid(Physics.FixedFrequency) &&
		   State.ConfigVersion == uint32(Physics.Version) && Sequence && Timebase && TimelineRevision &&
		   GeometryVersion == BallMotionLimits::GeometryVersion && FMath::IsFinite(ServerSeconds) &&
		   FMath::IsFinite(TimelineSeconds);
}
bool FBallAuthoritativeView::Accept(const FBallNetworkSnapshot &S)
{
	if (!S.IsValid() ||
		(bReady && (!FBallFinalityPolicy::IsSequenceNewer(S.Sequence, Latest.Sequence) ||
					(S.State.Identity.Epoch == Latest.State.Identity.Epoch &&
					 (S.State.Identity.BallId != Latest.State.Identity.BallId ||
					  S.State.Identity.Generation < Latest.State.Identity.Generation || S.Timebase < Latest.Timebase ||
					  (S.State.Identity.Generation == Latest.State.Identity.Generation &&
					   S.State.SimulationFrame < Latest.State.SimulationFrame))))))
		return false;
	Latest = S;
	bReady = true;
	return true;
}
namespace BallNetworkCodecDetail
{
enum class EWire : uint8
{
	Profile = 1,
	Poses = 2,
	Snapshot = 3,
	Remote = 4
};
void Header(FArchive &A, EWire Kind)
{
	uint32 Magic = 0x354e4248;
	uint16 Version = BallNet::Protocol;
	uint8 Type = uint8(Kind);
	A << Magic << Version << Type;
	if (Magic != 0x354e4248 || Version != BallNet::Protocol || Type != uint8(Kind))
		A.SetError();
}
void Vector(FArchive &A, FVector &V)
{
	A << V.X << V.Y << V.Z;
}
void Quat(FArchive &A, FQuat &Q)
{
	A << Q.X << Q.Y << Q.Z << Q.W;
}
void Material(FArchive &A, FBallSurfaceMaterial &M)
{
	A << M.bOverride << M.Restitution << M.Friction;
}
void Palm(FArchive &A, FBallPalmSettings &P)
{
	uint8 Shape = uint8(P.Shape);
	A << Shape;
	P.Shape = EBallPalmShape(Shape);
	Vector(A, P.BoxDimensions);
	A << P.RoundRadius << P.CCDDistanceTolerance << P.MaximumCCDIterations;
	A << P.RotationOffset.Pitch << P.RotationOffset.Yaw << P.RotationOffset.Roll << P.Radius;
	Vector(A, P.CenterOffset);
	Material(A, P.Material);
	A << P.FaceMaterials.bEnabled;
	Material(A, P.FaceMaterials.Front);
	Material(A, P.FaceMaterials.Back);
	Material(A, P.FaceMaterials.Side);
	A << P.bRotatingSurfaceFriction << P.MaximumCurveError;
}
void Tracking(FArchive &A, FBallHandTrackingSettings &S)
{
	A << S.MaximumSampleGap << S.MaximumExtrapolation << S.ExtrapolationFilterSeconds
	  << S.MaximumExtrapolationFilterError;
	A << S.MaximumSpeed << S.MaximumAngularSpeed << S.MaximumSampleRotation << S.MaximumAcceleration
	  << S.MaximumHeadSpeed << S.MaximumReach;
}
void Transport(FArchive &A, FBallTransportSettings &S)
{
	A << S.MaximumSamplesPerSecond;
	A << S.PoseRate << S.MinimumPoseRate << S.SnapshotRate << S.RemotePoseRate << S.MaximumRPCsPerSecond
	  << S.MaximumPoseAge;
	A << S.MaximumFuturePose << S.MaximumClockUncertainty << S.MaximumClockDriftPPM << S.OriginPositionTolerance
	  << S.OriginAngularTolerance << S.VisualDelay;
}
void Name(FArchive &A, FName &N)
{
	FString S = A.IsSaving() ? N.ToString() : FString();
	uint8 Count = uint8(FMath::Min(S.Len(), 64));
	A << Count;
	if (Count > 64 || (A.IsSaving() && S.Len() > 64))
	{
		A.SetError();
		return;
	}
	ANSICHAR Text[65] = {};
	if (A.IsSaving())
		for (int32 I = 0; I < Count; ++I)
		{
			if (S[I] > 127)
			{
				A.SetError();
				return;
			}
			Text[I] = ANSICHAR(S[I]);
		}
	A.Serialize(Text, Count);
	if (A.IsLoading() && !A.IsError())
		N = FName(ANSI_TO_TCHAR(Text));
}
void Physics(FArchive &A, FBallPhysicsConfig &C)
{
	A << C.Version << C.Radius << C.Mass;
	Vector(A, C.Gravity);
	A << C.LinearDrag << C.AngularDrag << C.Restitution << C.SurfaceFriction << C.SpinFriction << C.RollingResistance;
	A << C.MaximumLinearSpeed << C.MaximumAngularSpeed << C.SleepSpeed << C.WakeSpeed << C.SleepAngularSpeed
	  << C.SleepDwellSeconds;
	A << C.MinimumImpactSpeed << C.TOITolerance << C.ManifoldIterations;
	uint8 Channel = uint8(C.QueryChannel);
	A << Channel;
	C.QueryChannel = ECollisionChannel(Channel);
	Name(A, C.RequiredCollisionProfile);
	A << C.bTraceComplex << C.bUsePhysicalMaterials << C.MaximumWorldCandidates;
	A << C.CCDTolerance << C.PenetrationTolerance << C.DynamicContactReleaseDistance << C.MaximumCollisionIterations
	  << C.MaximumMicrointervals << C.FixedFrequency;
	if (Channel >= ECC_MAX)
		A.SetError();
}
void QuantPosition(FArchive &A, FVector &P)
{
	for (int32 I = 0; I < 3; ++I)
	{
		int32 N = 0;
		if (A.IsSaving())
		{
			const double Scaled = P[I] * 1000;
			if (!FMath::IsFinite(Scaled) || FMath::Abs(Scaled) > MAX_int32 - 1)
			{
				A.SetError();
				return;
			}
			N = FMath::RoundToInt32(Scaled);
		}
		A << N;
		if (A.IsLoading())
			P[I] = N * .001;
	}
}
void QuantRotation(FArchive &A, FQuat &Q)
{
	double Components[4] = {Q.X, Q.Y, Q.Z, Q.W};
	uint8 Largest = 0;
	if (A.IsSaving())
	{
		if (Q.ContainsNaN() || !Q.IsNormalized())
		{
			A.SetError();
			return;
		}
		for (int32 I = 1; I < 4; ++I)
			if (FMath::Abs(Components[I]) > FMath::Abs(Components[Largest]))
				Largest = I;
		if (Components[Largest] < 0)
			for (double &V : Components)
				V = -V;
	}
	A << Largest;
	if (Largest > 3)
	{
		A.SetError();
		return;
	}
	double Sum = 0;
	for (int32 I = 0; I < 4; ++I)
		if (I != Largest)
		{
			int16 N = A.IsSaving() ? int16(FMath::RoundToInt32(Components[I] * (32767 * UE_DOUBLE_SQRT_2))) : 0;
			A << N;
			if (A.IsLoading())
			{
				Components[I] = N / (32767 * UE_DOUBLE_SQRT_2);
				Sum += Components[I] * Components[I];
			}
		}
	if (A.IsLoading())
	{
		if (Sum > 1 + 1.e-9)
		{
			A.SetError();
			return;
		}
		Components[Largest] = FMath::Sqrt(FMath::Max(0.0, 1 - Sum));
		Q = FQuat(Components[0], Components[1], Components[2], Components[3]);
		Q.Normalize();
	}
}
void ProfileBody(FArchive &A, FBallPeerProfile &P)
{
	uint32 Geometry = BallMotionLimits::GeometryVersion;
	A << Geometry << P.Epoch << P.Participant << P.Version << P.Timebase << P.OriginRevision;
	A << P.EffectiveTimelineSeconds << P.EffectiveFrame;
	Palm(A, P.Left);
	Palm(A, P.Right);
	A << P.bEnableLeft << P.bEnableRight;
	Tracking(A, P.Tracking);
	Transport(A, P.Transport);
	Vector(A, P.Origin);
	Quat(A, P.OriginRotation);
	A << P.WorldToMeters;
	if (Geometry != BallMotionLimits::GeometryVersion)
		A.SetError();
}
void PoseBody(FArchive &A, FBallPoseBatch &B)
{
	A << B.Epoch << B.Timebase << B.ProfileVersion << B.OriginRevision << B.Sequence << B.CaptureGeneration
	  << B.SentSeconds;
	uint8 Count = uint8(B.Count);
	A << Count;
	if (Count > BallNet::MaximumPoseSamples || Count == 0 || (A.IsSaving() && B.Count != Count))
	{
		A.SetError();
		return;
	}
	B.Count = Count;
	// One reported origin per batch. It is still validated against the server-owned profile.
	FVector Origin = B.Samples[0].OriginPosition;
	FQuat OriginRotation = B.Samples[0].OriginOrientation;
	double WorldToMeters = B.Samples[0].WorldToMeters;
	QuantPosition(A, Origin);
	QuantRotation(A, OriginRotation);
	A << WorldToMeters;
	for (int32 I = 0; I < B.Count && !A.IsError(); ++I)
	{
		auto &P = B.Samples[I];
		uint8 Side = uint8(P.Side);
		A << Side << P.Sequence << P.CaptureSeconds << P.bTracked;
		uint32 Generation = B.Generations[I] ? B.Generations[I] : B.CaptureGeneration;
		A << Generation;
		B.Generations[I] = Generation;
		if (!Generation)
		{
			A.SetError();
			return;
		}
		P.Side = EBallHandSide(Side);
		P.Timebase = B.Timebase;
		P.ConfigurationVersion = B.ProfileVersion;
		P.OriginRevision = B.OriginRevision;
		P.ParticipantId = 1;
		QuantPosition(A, P.ControllerPosition);
		QuantRotation(A, P.ControllerOrientation);
		QuantPosition(A, P.HeadPosition);
		QuantRotation(A, P.HeadOrientation);
		if (A.IsSaving() &&
			(!P.OriginPosition.Equals(B.Samples[0].OriginPosition, 1.e-12) ||
			 !P.OriginOrientation.Equals(B.Samples[0].OriginOrientation, 1.e-12) || P.WorldToMeters != WorldToMeters))
		{
			A.SetError();
			return;
		}
		if (A.IsLoading())
		{
			P.OriginPosition = Origin;
			P.OriginOrientation = OriginRotation;
			P.WorldToMeters = WorldToMeters;
		}
		if (Side > 1 || !P.IsValid())
			A.SetError();
	}
	if (!B.Epoch.IsValid() || !B.Timebase || !B.ProfileVersion || !B.OriginRevision || !B.Sequence ||
		!B.CaptureGeneration || !FMath::IsFinite(B.SentSeconds))
		A.SetError();
}
void SnapshotBody(FArchive &A, FBallNetworkSnapshot &S)
{
	A << S.Sequence << S.Timebase << S.TimelineRevision << S.GeometryVersion << S.ServerSeconds << S.TimelineSeconds
	  << S.bPaused;
	Physics(A, S.Physics);
	A << S.History.RevisionWindowSeconds << S.History.HistoryCapacity << S.History.MaximumReplayFrames;
	TArray<uint8> State;
	if (A.IsSaving() && !FBallStateCodec::Save(S.State, State))
	{
		A.SetError();
		return;
	}
	uint16 Count = uint16(State.Num());
	A << Count;
	if (Count > FBallStateCodec::MaximumEncodedBytes || Count == 0)
	{
		A.SetError();
		return;
	}
	if (A.IsLoading())
		State.SetNum(Count);
	A.Serialize(State.GetData(), Count);
	if (A.IsLoading() && !A.IsError() && !FBallStateCodec::Load(State, S.State))
		A.SetError();
}
template <class T, class Fn> bool Save(T Value, EWire Kind, FBallWirePacket &Out, Fn Body)
{
	FBallWirePacket Packet;
	Packet.Bytes.Reserve(2048);
	FMemoryWriter W(Packet.Bytes, true);
	Header(W, Kind);
	Body(W, Value);
	if (W.IsError() || Packet.Bytes.Num() > BallNet::MaximumPacketBytes)
		return false;
	Out = MoveTemp(Packet);
	return true;
}
template <class T, class Fn> bool Load(const FBallWirePacket &Packet, EWire Kind, T &Out, Fn Body)
{
	if (Packet.Bytes.Num() < 7 || Packet.Bytes.Num() > BallNet::MaximumPacketBytes)
		return false;
	T Value;
	FMemoryReader R(Packet.Bytes, true);
	Header(R, Kind);
	if (R.IsError())
		return false;
	Body(R, Value);
	if (R.IsError() || R.Tell() != R.TotalSize())
		return false;
	Out = MoveTemp(Value);
	return true;
}
} // namespace BallNetworkCodecDetail
bool FBallNetworkCodec::WriteProfile(const FBallPeerProfile &P, FBallWirePacket &Out)
{
	using namespace BallNetworkCodecDetail;
	return P.IsValid() && Save(P, EWire::Profile, Out, ProfileBody);
}
bool FBallNetworkCodec::ReadProfile(const FBallWirePacket &P, FBallPeerProfile &Out)
{
	using namespace BallNetworkCodecDetail;
	FBallPeerProfile V;
	if (!Load(P, EWire::Profile, V, ProfileBody) || !V.IsValid())
		return false;
	Out = V;
	return true;
}
bool FBallNetworkCodec::WritePoses(const FBallPoseBatch &B, FBallWirePacket &Out, bool Remote)
{
	using namespace BallNetworkCodecDetail;
	return Save(B, Remote ? EWire::Remote : EWire::Poses, Out, PoseBody);
}
bool FBallNetworkCodec::ReadPoses(const FBallWirePacket &P, FBallPoseBatch &Out, bool Remote)
{
	using namespace BallNetworkCodecDetail;
	return Load(P, Remote ? EWire::Remote : EWire::Poses, Out, PoseBody);
}
bool FBallNetworkCodec::WriteSnapshot(const FBallNetworkSnapshot &S, FBallWirePacket &Out)
{
	using namespace BallNetworkCodecDetail;
	return S.IsValid() && Save(S, EWire::Snapshot, Out, SnapshotBody);
}
bool FBallNetworkCodec::ReadSnapshot(const FBallWirePacket &P, FBallNetworkSnapshot &Out)
{
	using namespace BallNetworkCodecDetail;
	FBallNetworkSnapshot V;
	if (!Load(P, EWire::Snapshot, V, SnapshotBody) || !V.IsValid())
		return false;
	Out = V;
	return true;
}
