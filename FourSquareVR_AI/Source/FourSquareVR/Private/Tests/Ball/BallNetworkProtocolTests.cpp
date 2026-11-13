#include "Misc/AutomationTest.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "BallTestFixtures.h"
#include "Ball/BallNetworkTypes.h"
#include "Ball/BallNetworkClock.h"
#include "Ball/BallPoseAdmission.h"
#include "Ball/BallPalmGeometry.h"
#include "Ball/BallHandCollisionSolver.h"
#include "HAL/PlatformTime.h"
#include <limits>
namespace
{
constexpr auto NetworkProtocolFlags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;
FBallPeerProfile NetProfile()
{
	FBallPeerProfile P;
	P.Epoch = BallTests::State().Identity.Epoch;
	P.Participant = 5;
	return P;
}
FBallPoseBatch NetBatch(double Time = 1)
{
	FBallPoseBatch B;
	B.Epoch = NetProfile().Epoch;
	B.SentSeconds = Time;
	B.Count = 1;
	B.Samples[0].CaptureSeconds = Time;
	B.Samples[0].Sequence = 1;
	B.Samples[0].HeadPosition = FVector(0, 0, 50);
	return B;
}
FBallClockEstimate NetClock()
{
	FBallClockEstimate E;
	E.Version = 1;
	E.Uncertainty = .0001;
	return E;
}
} // namespace
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetProfileCodec, "FourSquareVR.Ball.Network.ProfileRoundTrip", NetworkProtocolFlags)
bool FNetProfileCodec::RunTest(const FString &)
{
	auto P = NetProfile();
	P.Left.CenterOffset = FVector(1.234, 2.345, 3.456);
	P.Left.RotationOffset = FRotator(7, 13, 29);
	P.Right.Shape = EBallPalmShape::Sphere;
	P.Right.Radius = 6.8;
	P.Left.FaceMaterials.bEnabled = true;
	FBallWirePacket W;
	FBallPeerProfile Q;
	TestTrue(TEXT("Encode authoritative profile"), FBallNetworkCodec::WriteProfile(P, W));
	TestTrue(TEXT("Decode profile"), FBallNetworkCodec::ReadProfile(W, Q));
	TestTrue(TEXT("Independent calibration and geometry exact"),
			 P.Left == Q.Left && P.Right == Q.Right && P.Tracking == Q.Tracking);
	TestTrue(TEXT("Identity/context preserved"),
			 Q.Participant == P.Participant && Q.Epoch == P.Epoch && Q.Timebase == P.Timebase);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetPosePrecision, "FourSquareVR.Ball.Network.PoseQuantizationErrorAndSigns",
								 NetworkProtocolFlags)
bool FNetPosePrecision::RunTest(const FString &)
{
	FRandomStream R(51201);
	double MaxPosition = 0, MaxAngle = 0;
	for (int32 I = 0; I < 1024; ++I)
	{
		auto B = NetBatch();
		auto &P = B.Samples[0];
		P.ControllerPosition =
			FVector(R.FRandRange(-10000, 10000), R.FRandRange(-10000, 10000), R.FRandRange(-10000, 10000));
		P.ControllerOrientation =
			FRotator(R.FRandRange(-180, 180), R.FRandRange(-180, 180), R.FRandRange(-180, 180)).Quaternion();
		if (I % 2)
			P.ControllerOrientation = P.ControllerOrientation * -1;
		FBallWirePacket W;
		FBallPoseBatch Q;
		if (!FBallNetworkCodec::WritePoses(B, W) || !FBallNetworkCodec::ReadPoses(W, Q))
		{
			AddError(TEXT("Valid pose round trip"));
			return false;
		}
		MaxPosition = FMath::Max(MaxPosition, (P.ControllerPosition - Q.Samples[0].ControllerPosition).Size());
		MaxAngle = FMath::Max(
			MaxAngle,
			FBallPalmGeometry::RotationVector(P.ControllerOrientation, Q.Samples[0].ControllerOrientation).Size());
	}
	TestTrue(TEXT("Quantized position <= sqrt(3)*0.0005 cm"), MaxPosition <= .000867);
	TestTrue(TEXT("Angular error < .005 degree"), MaxAngle < FMath::DegreesToRadians(.005));
	AddInfo(FString::Printf(TEXT("POSE_CODEC Seed=51201 Samples=1024 MaxPositionCm=%.9f MaxAngleRadians=%.9f"),
							MaxPosition, MaxAngle));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetMalformed, "FourSquareVR.Ball.Network.MalformedPacketsBoundedAndAtomic",
								 NetworkProtocolFlags)
bool FNetMalformed::RunTest(const FString &)
{
	FBallWirePacket W;
	FBallNetworkCodec::WritePoses(NetBatch(), W);
	for (int32 I = 0; I < W.Bytes.Num(); ++I)
	{
		auto Bad = W;
		Bad.Bytes.SetNum(I);
		FBallPoseBatch Out;
		Out.Sequence = 99;
		TestFalse(TEXT("Truncated payload rejected"), FBallNetworkCodec::ReadPoses(Bad, Out));
		TestEqual(TEXT("Caller unchanged on failure"), Out.Sequence, uint32(99));
	}
	auto Bad = W;
	Bad.Bytes[4] ^= 64;
	FBallPoseBatch Out;
	TestFalse(TEXT("Unknown version"), FBallNetworkCodec::ReadPoses(Bad, Out));
	Bad = W;
	Bad.Bytes.Add(1);
	TestFalse(TEXT("Trailing data"), FBallNetworkCodec::ReadPoses(Bad, Out));
	Bad.Bytes.SetNum(BallNet::MaximumPacketBytes + 1);
	TestFalse(TEXT("Oversized packet refused before parsing"), FBallNetworkCodec::ReadPoses(Bad, Out));
	auto B = NetBatch();
	B.Count = BallNet::MaximumPoseSamples + 1;
	TestFalse(TEXT("Bounded sender count"), FBallNetworkCodec::WritePoses(B, Bad));
	B = NetBatch();
	B.Samples[0].ControllerOrientation = FQuat(0, 0, 0, 0);
	TestFalse(TEXT("Invalid quaternion"), FBallNetworkCodec::WritePoses(B, Bad));
	B = NetBatch();
	B.Samples[0].ControllerPosition.X = 1.e12;
	TestFalse(TEXT("Quantization range never wraps"), FBallNetworkCodec::WritePoses(B, Bad));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetSnapshotCodec, "FourSquareVR.Ball.Network.CompleteSnapshotRoundTripAndOrdering",
								 NetworkProtocolFlags)
bool FNetSnapshotCodec::RunTest(const FString &)
{
	FBallNetworkSnapshot S;
	S.State = BallTests::State();
	S.State.LinearVelocity = FVector(12, 34, 56);
	S.State.NextEventOrdinal = 9;
	S.State.MotionPairCount = 1;
	S.State.MotionPairs[0].Collider = BallMotionLimits::HandId(5, EBallHandSide::Left, 2);
	S.State.MotionPairs[0].Episode = 3;
	S.State.MotionPairs[0].bArmed = true;
	FBallWirePacket W;
	FBallNetworkSnapshot Q;
	TestTrue(TEXT("Encode"), FBallNetworkCodec::WriteSnapshot(S, W));
	TestTrue(TEXT("Decode"), FBallNetworkCodec::ReadSnapshot(W, Q));
	TestTrue(TEXT("All checkpoint fields survive"), BallTests::ExactlyEqual(S.State, Q.State));
	FBallAuthoritativeView V;
	TestTrue(TEXT("Initial baseline"), V.Accept(Q));
	TestFalse(TEXT("Duplicate rejected"), V.Accept(Q));
	Q.Sequence = 2;
	Q.State.SimulationFrame = 1;
	TestTrue(TEXT("New authoritative state"), V.Accept(Q));
	Q.Sequence = 3;
	Q.State.SimulationFrame = 0;
	TestFalse(TEXT("Regressing same-generation frame"), V.Accept(Q));
	Q.State.Identity.Generation++;
	TestTrue(TEXT("Reset may change state at boundary"), V.Accept(Q));
	Q.Sequence = 2;
	TestFalse(TEXT("Old packet after reset cannot overwrite"), V.Accept(Q));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetClockSymmetric, "FourSquareVR.Ball.Network.ClockOffsetRTTJitterAndAsymmetry",
								 NetworkProtocolFlags)
bool FNetClockSymmetric::RunTest(const FString &)
{
	FBallNetworkClock C;
	for (int32 I = 0; I < 8; ++I)
		TestTrue(TEXT("Four stamp exchange"), C.Observe(100 + I, 105.02 + I, 105.021 + I, 100.041 + I));
	const auto E = C.GetEstimate(107.05);
	TestTrue(TEXT("Clock ready"), C.IsReady());
	TestEqual(TEXT("Offset"), E.RemoteMinusLocal, 5.0, 1.e-9);
	TestEqual(TEXT("Measured RTT excludes remote processing"), E.RTT, .04, 1.e-9);
	TestEqual(TEXT("Map capture, not arrival"), E.RemoteToLocal(106), 101.0, 1.e-9);
	FBallNetworkClock A;
	A.Observe(100, 105.01, 105.011, 100.101);
	A.Observe(101, 106.01, 106.011, 101.101);
	const auto Asym = A.GetEstimate(101.101);
	TestTrue(TEXT("Asymmetric bias included in uncertainty"),
			 FMath::Abs(Asym.RemoteMinusLocal - 5) <= Asym.Uncertainty);
	TestTrue(TEXT("Cannot claim RTT/2 is exact forward delay"), FMath::Abs(Asym.RemoteMinusLocal - 5) > .03);
	TestFalse(TEXT("Forged processing time"), A.Observe(102, 107, 108, 102.1));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetClockDrift, "FourSquareVR.Ball.Network.ClockDriftAndDiscontinuity",
								 NetworkProtocolFlags)
bool FNetClockDrift::RunTest(const FString &)
{
	FBallNetworkClock C;
	const double Drift = .0001;
	for (int32 I = 0; I < 80; ++I)
	{
		const double A = 10 + I * .5, D = A + .021;
		TestTrue(TEXT("Drifting clock sample"),
				 C.Observe(A, (A + .01) * (1 + Drift) + 5, (A + .011) * (1 + Drift) + 5, D));
	}
	const auto E = C.GetEstimate(50);
	TestEqual(TEXT("100 ppm estimated"), E.Drift, Drift, 5.e-6);
	TestTrue(TEXT("Mapped clock error within uncertainty"),
			 FMath::Abs(E.RemoteToLocal(50 * (1 + Drift) + 5) - 50) <= E.Uncertainty);
	TestFalse(TEXT("Abrupt clock jump is not silently admitted"), C.Observe(51, 71, 71.001, 51.021));
	TestFalse(TEXT("Reordered clock measurement"), C.Observe(1, 6, 6.001, 1.021));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetAdmissionValidation,
								 "FourSquareVR.Ball.Network.TimestampOriginAndContextValidation", NetworkProtocolFlags)
bool FNetAdmissionValidation::RunTest(const FString &)
{
	FBallPoseAdmission A;
	A.Initialize(NetProfile());
	auto B = NetBatch();
	auto C = NetClock();
	TestTrue(TEXT("Valid admitted measurement"), A.Admit(B, C, true, 1, 1) == EBallInputDisposition::Accepted);
	TestTrue(TEXT("Redundancy duplicate no impulse"),
			 A.Admit(B, C, true, 1.001, 1.001) == EBallInputDisposition::Duplicate);
	B.Samples[0].ControllerPosition.X = 1;
	TestTrue(TEXT("Conflicting duplicate"),
			 A.Admit(B, C, true, 1.002, 1.002) == EBallInputDisposition::ConflictingDuplicate);
	A.Initialize(NetProfile());
	B = NetBatch();
	B.Timebase = 2;
	TestTrue(TEXT("Wrong timeline"), A.Admit(B, C, true, 1, 1) == EBallInputDisposition::WrongContext);
	B = NetBatch(.4);
	TestTrue(TEXT("Old pose"), A.Admit(B, C, true, 1, 1) == EBallInputDisposition::TooOld);
	B = NetBatch(1.1);
	TestTrue(TEXT("Future pose"), A.Admit(B, C, true, 1, 1) == EBallInputDisposition::Future);
	B = NetBatch();
	B.Samples[0].OriginPosition.X = 10;
	TestTrue(TEXT("Unapproved origin jump"), A.Admit(B, C, true, 1, 1) == EBallInputDisposition::InvalidOrigin);
	A.Initialize(NetProfile());
	C.Uncertainty = .2;
	B = NetBatch();
	TestTrue(TEXT("Clock uncertainty explicit"), A.Admit(B, C, true, 1, 1) == EBallInputDisposition::ClockUncertain);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetPoseHistory, "FourSquareVR.Ball.Network.ReorderedHistoryFrozenTimesAndPinning",
								 NetworkProtocolFlags)
bool FNetPoseHistory::RunTest(const FString &)
{
	FBallNetworkPoseHistory H;
	H.Initialize();
	FBallHandTrackingSettings T;
	auto P = NetBatch().Samples[0];
	P.CaptureSeconds = P.TimelineSeconds = 0;
	P.Sequence = 1;
	TestTrue(TEXT("First warmup"), H.Insert(P, 1, T) == EBallInputDisposition::Accepted);
	P.CaptureSeconds = P.TimelineSeconds = .02;
	P.Sequence = 3;
	P.ControllerPosition.X = 2;
	H.Insert(P, 1, T);
	P.CaptureSeconds = P.TimelineSeconds = .01;
	P.Sequence = 2;
	P.ControllerPosition.X = 1;
	TestTrue(TEXT("Late sample stored in chronological history"),
			 H.Insert(P, 1, T) == EBallInputDisposition::LateHistorical);
	TestEqual(TEXT("Correct interpolated velocity"), H.View().Last().LinearVelocity.X, 100.0, 1.e-9);
	P.TimelineSeconds = .5;
	TestTrue(TEXT("Duplicate never remaps frozen capture time"), H.Insert(P, 1, T) == EBallInputDisposition::Duplicate);
	const SIZE_T Bytes = H.GetAllocatedBytes();
	for (int32 I = 3; I < 259; ++I)
	{
		P.Sequence = I + 1;
		P.CaptureSeconds = P.TimelineSeconds = I * .01;
		P.ControllerPosition.X = I;
		H.Insert(P, 1, T);
	}
	TestEqual(TEXT("Bounded raw ring"), H.Num(), 256);
	TestEqual(TEXT("No capacity growth"), H.GetAllocatedBytes(), Bytes);
	H.PinFrom(0);
	P.Sequence = 260;
	P.CaptureSeconds = P.TimelineSeconds = 2.59;
	P.ControllerPosition.X = 259;
	TestTrue(TEXT("Pinned input cannot be overwritten"), H.Insert(P, 1, T) == EBallInputDisposition::Capacity);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetRateLimit, "FourSquareVR.Ball.Network.RateLimitAndPacketLossDiagnostics",
								 NetworkProtocolFlags)
bool FNetRateLimit::RunTest(const FString &)
{
	FBallPoseAdmission A;
	A.Initialize(NetProfile());
	auto C = NetClock();
	int32 Rejected = 0;
	for (int32 I = 0; I < 20; ++I)
	{
		auto B = NetBatch();
		B.Sequence = I + 1;
		if (A.Admit(B, C, true, 1, 1) == EBallInputDisposition::RateLimited)
			++Rejected;
	}
	TestEqual(TEXT("Burst bounded"), Rejected, 12);
	A.Initialize(NetProfile());
	auto B = NetBatch();
	A.Admit(B, C, true, 1, 1);
	B.Sequence = 4;
	A.Admit(B, C, true, 1.1, 1.1);
	TestEqual(TEXT("Sequence gaps measured separately from physical poses"), A.Statistics.SequenceGaps, uint64(2));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetUniqueSampleBudget,
								 "FourSquareVR.Ball.Network.UniqueSampleBudgetAndPolicyCapacity", NetworkProtocolFlags)
bool FNetUniqueSampleBudget::RunTest(const FString &)
{
	FBallPoseAdmission A;
	A.Initialize(NetProfile());
	auto B = NetBatch();
	B.Count = 12;
	for (int32 I = 0; I < 12; ++I)
	{
		B.Samples[I] = NetBatch().Samples[0];
		B.Samples[I].Sequence = I + 1;
		B.Samples[I].CaptureSeconds = 1 + I * .00001;
	}
	B.SentSeconds = 1.001;
	TestTrue(TEXT("Bounded unique burst"), A.Admit(B, NetClock(), true, 1, 1) == EBallInputDisposition::Accepted);
	TestTrue(TEXT("Redundant poses do not consume unique budget"),
			 A.Admit(B, NetClock(), true, 1, 1) == EBallInputDisposition::Duplicate);
	++B.Sequence;
	for (auto &P : B.Samples)
	{
		P.Sequence += 12;
		P.CaptureSeconds += .00012;
	}
	TestTrue(TEXT("Excess samples rejected despite valid RPC rate"),
			 A.Admit(B, NetClock(), true, 1, 1) == EBallInputDisposition::RateLimited);
	TestEqual(TEXT("Per-record atomic bounded burst retains 16 records"), A.Hand(EBallHandSide::Left).Num(), 16);
	auto P = NetProfile();
	P.Transport.MaximumPoseAge = 2;
	TestFalse(TEXT("Rate/age combination cannot promise more raw retention than allocated"), P.IsValid());
	P.Transport.MaximumSamplesPerSecond = 120;
	TestTrue(TEXT("Explicit compatible rate/age is valid"), P.IsValid());
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetProfileActivation, "FourSquareVR.Ball.Network.ProfileActivationCannotBeBackdated",
								 NetworkProtocolFlags)
bool FNetProfileActivation::RunTest(const FString &)
{
	auto P = NetProfile();
	P.EffectiveFrame = 120;
	P.EffectiveTimelineSeconds = 1;
	FBallWirePacket W;
	FBallPeerProfile Q;
	TestTrue(TEXT("Server activation encoded"),
			 FBallNetworkCodec::WriteProfile(P, W) && FBallNetworkCodec::ReadProfile(W, Q));
	TestEqual(TEXT("Activation timeline preserved"), Q.EffectiveTimelineSeconds, 1.0);
	TestEqual(TEXT("Activation frame preserved"), Q.EffectiveFrame, uint64(120));
	FBallPoseAdmission A;
	A.Initialize(Q);
	auto B = NetBatch(.99);
	TestTrue(TEXT("New calibration cannot reinterpret earlier tracking"),
			 A.Admit(B, NetClock(), true, 1.01, 1.01) == EBallInputDisposition::WrongContext);
	TestEqual(TEXT("Rejected backdating keeps accepted history empty"), A.Hand(EBallHandSide::Left).Num(), 0);
	B = NetBatch(1.02);
	TestTrue(TEXT("Post-activation observation admitted"),
			 A.Admit(B, NetClock(), true, 1.03, 1.03) == EBallInputDisposition::Accepted);
	return true;
}
#endif
