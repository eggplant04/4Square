#include "Misc/AutomationTest.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "BallPalmTestFixtures.h"
#include "Ball/BallPoseAdmission.h"
#include "Ball/BallNetworkComponent.h"
#include "Ball/VRHandCollisionComponent.h"
#include "HAL/PlatformTime.h"
namespace BallNetworkPhysicsTests
{
constexpr auto NetworkPhysicsTestFlags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;
inline FBallPeerProfile Profile()
{
	FBallPeerProfile P;
	P.Epoch = BallTests::State().Identity.Epoch;
	P.Participant = 1;
	P.Left.BoxDimensions = FVector(12, 1, 1);
	P.Left.RoundRadius = .2;
	P.Left.Material.bOverride = true;
	P.Left.Material.Friction = 0;
	return P;
}
inline FBallClockEstimate Clock()
{
	FBallClockEstimate E;
	E.Version = 1;
	E.Uncertainty = .0001;
	return E;
}
inline FBallPoseBatch Batch(const FBallPeerProfile &P, const FBallHandObservation &O, uint32 Gen = 1)
{
	FBallPoseBatch B;
	B.Epoch = P.Epoch;
	B.Timebase = P.Timebase;
	B.ProfileVersion = P.Version;
	B.OriginRevision = P.OriginRevision;
	B.Sequence = O.Sequence;
	B.SentSeconds = O.CaptureSeconds;
	B.Count = 1;
	B.Samples[0] = O;
	B.Generations[0] = Gen;
	return B;
}
inline bool Feed(FBallPoseAdmission &A, const FBallHandObservation &O, uint32 Gen = 1)
{
	FBallWirePacket W;
	FBallPoseBatch B;
	if (!FBallNetworkCodec::WritePoses(Batch(A.Profile, O, Gen), W) || !FBallNetworkCodec::ReadPoses(W, B))
		return false;
	const auto R = A.Admit(B, Clock(), true, O.CaptureSeconds + .01, O.CaptureSeconds + .01);
	return R == EBallInputDisposition::Accepted || R == EBallInputDisposition::Duplicate;
}
inline FBallHandObservation Wrist(int32 I)
{
	auto P = PalmTests::Pose(.1 + I / 120.0, FVector::ZeroVector, FQuat(FVector::UpVector, -.9 + I * .6), I + 1);
	return P;
}
inline FBallMotionFrame Frame(FBallPoseAdmission &A, double Time = .1 + 1.0 / 120)
{
	FBallMotionFrame F;
	F.Epoch = A.Profile.Epoch;
	F.Frame = 1;
	A.BuildMotion(Time, F);
	return F;
}
} // namespace BallNetworkPhysicsTests
using namespace BallNetworkPhysicsTests;
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetworkWristCCD, "FourSquareVR.Ball.Network.Physics.QuantizedWristOnlyTOIAndImpulse",
								 NetworkPhysicsTestFlags)
bool FNetworkWristCCD::RunTest(const FString &)
{
	FBallPoseAdmission A;
	A.Initialize(Profile());
	for (int32 I = 0; I < 3; ++I)
		TestTrue(TEXT("Accepted quaternion observation"), Feed(A, Wrist(I)));
	auto F = Frame(A);
	if (!TestEqual(TEXT("One rotating collider"), F.ColliderCount, 1))
		return false;
	auto Q = PalmTests::Query(FVector(5, 0, 0), FVector(5, 0, 0), .5);
	FBallContact Hit;
	TestTrue(TEXT("Network-reconstructed rotating CCD"),
			 FBallHandCollisionSolver::Query(F, F.Colliders[0], Q, Hit) == EBallQueryResult::Contact);
	const double Expected = (-FMath::Asin(.2) + .3) / .6;
	const double Error = FMath::Abs(Hit.TimeOfImpact - Expected) / 120;
	TestTrue(TEXT("Quantized contact time within 2 microseconds"), Error < 2.e-6);
	auto S = BallTests::State(1);
	S.Position = FVector(5, 0, 0);
	auto C = PalmTests::Config();
	C.Radius = .5;
	FBallStepOutput O;
	TestTrue(TEXT("Actual authoritative wrist step"), PalmTests::Step(F, S, C, O) == EBallStepResult::Success);
	if (!TestTrue(TEXT("Exactly one wrist impulse"), O.EventCount == 1))
		return false;
	const double Angle = -FMath::Asin(.2);
	const FVector Normal(-FMath::Sin(Angle), FMath::Cos(Angle), 0);
	const FVector V = FVector::CrossProduct(FVector(0, 0, 72), FVector(5, 0, 0) - Normal * .5);
	const FVector ExpectedVelocity = Normal * (1.8 * FVector::DotProduct(V, Normal));
	const double VelocityError = (O.Events[0].OutgoingVelocity - ExpectedVelocity).Size();
	TestTrue(TEXT("Quantized impulse within .1 cm/s"), VelocityError < .1);
	AddInfo(FString::Printf(TEXT("NET_WRIST TOIErrorSeconds=%.9f VelocityErrorCmS=%.6f"), Error, VelocityError));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetworkTranslation,
								 "FourSquareVR.Ball.Network.Physics.FastTranslatedPalmNoTunnelling",
								 NetworkPhysicsTestFlags)
bool FNetworkTranslation::RunTest(const FString &)
{
	auto P = Profile();
	P.Left.BoxDimensions = FVector(8, 12, 2);
	P.Left.RoundRadius = .5;
	FBallPoseAdmission A;
	A.Initialize(P);
	for (int32 I = 0; I < 3; ++I)
	{
		auto O = PalmTests::Pose(.1 + I / 120.0, FVector(0, 0, -30 + 20 * I), FQuat::Identity, I + 1);
		O.HeadPosition = FVector(0, 0, 50);
		TestTrue(TEXT("Fast legitimate hand accepted"), Feed(A, O));
	}
	auto F = Frame(A);
	if (!TestEqual(TEXT("Collider"), F.ColliderCount, 1))
		return false;
	FBallContact H;
	auto Q = PalmTests::Query(FVector::ZeroVector, FVector::ZeroVector, .5);
	TestTrue(TEXT("Moving palm hits stationary ball"),
			 FBallHandCollisionSolver::Query(F, F.Colliders[0], Q, H) == EBallQueryResult::Contact);
	TestEqual(TEXT("Analytic fraction"), H.TimeOfImpact, .425, 1.e-6);
	TestTrue(TEXT("Normal +Z"), H.Normal.Equals(FVector::UpVector, 1.e-6));
	TestEqual(TEXT("Relative surface velocity"), H.SurfaceVelocity.Z, 2400.0, 1.e-5);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetworkReplay,
								 "FourSquareVR.Ball.Network.Physics.HistoricalReplayIgnoresCurrentControllerAndClock",
								 NetworkPhysicsTestFlags)
bool FNetworkReplay::RunTest(const FString &)
{
	FBallPoseAdmission A;
	A.Initialize(Profile());
	for (int32 I = 0; I < 3; ++I)
		Feed(A, Wrist(I));
	auto F = Frame(A);
	auto S = BallTests::State(1);
	S.Position = FVector(5, 0, 0);
	auto C = PalmTests::Config();
	C.Radius = .5;
	auto First = S, Second = S;
	FBallStepOutput O;
	TestTrue(TEXT("First simulation"), PalmTests::Step(F, First, C, O) == EBallStepResult::Success);
	// Later observations cannot change the already frozen motion input.
	for (int32 I = 3; I < 8; ++I)
		Feed(A, Wrist(I));
	TestTrue(TEXT("Restored simulation with recorded input"),
			 PalmTests::Step(F, Second, C, O) == EBallStepResult::Success);
	TestTrue(TEXT("Complete result identical"), BallTests::ExactlyEqual(First, Second));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetworkIndependentHands,
								 "FourSquareVR.Ball.Network.Physics.FourPlayersAndIndependentTrackingGenerations",
								 NetworkPhysicsTestFlags)
bool FNetworkIndependentHands::RunTest(const FString &)
{
	FBallMotionFrame F;
	F.Epoch = Profile().Epoch;
	for (uint32 Player = 1; Player <= 4; ++Player)
	{
		auto P = Profile();
		P.Participant = Player;
		FBallPoseAdmission A;
		A.Initialize(P);
		for (int32 I = 0; I < 4; ++I)
			for (uint8 Side = 0; Side < 2; ++Side)
			{
				auto O = PalmTests::Pose(.1 + I / 120.0, FVector(20 * Player, 20 * Side, 0), FQuat::Identity, I + 1);
				O.Side = EBallHandSide(Side);
				TestTrue(TEXT("Both hands independent"), Feed(A, O, Side ? 1 : (I < 1 ? 1 : 2)));
			}
		A.BuildMotion(.1 + 2.0 / 120, F);
	}
	TestEqual(TEXT("All eight accepted palms coexist"), F.ColliderCount, 8);
	for (int32 I = 0; I < F.ColliderCount; ++I)
		for (int32 J = I + 1; J < F.ColliderCount; ++J)
			TestFalse(TEXT("Stable unique participant/side identity"), F.Colliders[I].Id == F.Colliders[J].Id);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetworkLossRecovery,
								 "FourSquareVR.Ball.Network.Physics.MissingLostAndReacquiredTracking",
								 NetworkPhysicsTestFlags)
bool FNetworkLossRecovery::RunTest(const FString &)
{
	FBallPoseAdmission A;
	A.Initialize(Profile());
	for (int32 I = 0; I < 3; ++I)
		Feed(A, Wrist(I));
	TestEqual(TEXT("No invented trajectory after extrapolation budget"), Frame(A, .2).ColliderCount, 0);
	auto O = Wrist(3);
	O.bTracked = false;
	Feed(A, O, 2);
	TestEqual(TEXT("Tracking loss has no current collider"), Frame(A, O.CaptureSeconds).ColliderCount, 0);
	for (int32 I = 4; I < 7; ++I)
	{
		O = Wrist(I);
		O.ControllerOrientation = FQuat::Identity;
		Feed(A, O, 3);
	}
	TestEqual(TEXT("Only continuous reacquired motion returns"), Frame(A, .1 + 5.0 / 120).ColliderCount, 1);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetworkInvalidMotion,
								 "FourSquareVR.Ball.Network.Physics.InvalidAngularVelocityAndOriginBarrier",
								 NetworkPhysicsTestFlags)
bool FNetworkInvalidMotion::RunTest(const FString &)
{
	FBallPoseAdmission A;
	A.Initialize(Profile());
	Feed(A, Wrist(0));
	auto O = Wrist(1);
	O.CaptureSeconds = O.TimelineSeconds = .1001;
	auto B = Batch(A.Profile, O);
	TestTrue(TEXT("Forged angular spike refused"),
			 A.Admit(B, Clock(), true, .11, .11) == EBallInputDisposition::InvalidMotion);
	TestEqual(TEXT("Invalid motion cannot mutate accepted history"), A.Hand(EBallHandSide::Left).Num(), 1);
	auto P = A.Profile;
	++P.Version;
	++P.OriginRevision;
	P.Origin.X = 100;
	A.ChangeProfile(P);
	TestEqual(TEXT("Origin transition preserves historical observations"), A.Hand(EBallHandSide::Left).Num(), 1);
	B = Batch(Profile(), Wrist(1));
	TestTrue(TEXT("Old origin cannot be applied after authority boundary"),
			 A.Admit(B, Clock(), true, .12, .12) == EBallInputDisposition::WrongContext);
	TestEqual(TEXT("No swept teleport"), Frame(A).ColliderCount, 0);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetworkProfileCosts,
								 "FourSquareVR.Ball.Network.Performance.SerializationValidationAndMemory",
								 NetworkPhysicsTestFlags)
bool FNetworkProfileCosts::RunTest(const FString &)
{
	FBallPoseAdmission A;
	A.Initialize(Profile());
	FBallWirePacket W;
	auto B = Batch(Profile(), Wrist(0));
	B.Count = 12;
	for (int32 I = 0; I < 12; ++I)
	{
		B.Samples[I] = Wrist(I % 6);
		B.Samples[I].Side = EBallHandSide(I / 6);
		B.Generations[I] = 1;
	}
	B.SentSeconds = 1;
	const double Begin = FPlatformTime::Seconds();
	for (int32 I = 0; I < 2000; ++I)
	{
		FBallPoseBatch Q;
		if (!FBallNetworkCodec::WritePoses(B, W) || !FBallNetworkCodec::ReadPoses(W, Q))
			return false;
	}
	const double Us = (FPlatformTime::Seconds() - Begin) * 1.e6 / 2000;
	FBallNetworkSnapshot S;
	S.State = BallTests::State();
	FBallWirePacket Snapshot;
	FBallNetworkCodec::WriteSnapshot(S, Snapshot);
	FBallMotionHistory H;
	H.Initialize(256);
	TestEqual(TEXT("Padding reduction retained"), sizeof(FBallMotionSegment), SIZE_T(160));
	TestEqual(TEXT("Full history capacity retained"), H.GetAllocatedBytes(), SIZE_T(4882432));
	AddInfo(FString::Printf(TEXT("NET_COST CodecRoundTripUs=%.3f PoseBytes=%d SnapshotBytes=%d Pose30HzBps=%d "
								 "Pose60HzBps=%d Pose120HzBps=%d AcceptedHistoryPerPlayer=%llu ComponentStatic=%llu "
								 "BallBaseline16=%llu FullMotionHistory=%llu"),
							Us, W.Bytes.Num(), Snapshot.Bytes.Num(), W.Bytes.Num() * 30, W.Bytes.Num() * 60,
							W.Bytes.Num() * 120, uint64(A.GetAllocatedBytes()),
							uint64(sizeof(UVRHandCollisionComponent)), uint64(16 * sizeof(FBallNetworkSnapshot)),
							uint64(H.GetAllocatedBytes())));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetworkDelayedLoss,
								 "FourSquareVR.Ball.Network.Timing.DelayedLostReorderedInputsStayHistorical",
								 NetworkPhysicsTestFlags)
bool FNetworkDelayedLoss::RunTest(const FString &)
{
	FBallPoseAdmission A;
	A.Initialize(Profile());
	auto C = Clock();
	C.RemoteMinusLocal = 5;
	C.Uncertainty = .02;
	for (int32 Packet = 0; Packet < 10; ++Packet)
	{
		if (Packet == 3 || Packet == 6)
			continue; // Reproducible burst losses; six redundant captures bridge these gaps.
		FBallPoseBatch B;
		B.Epoch = A.Profile.Epoch;
		B.Sequence = Packet + 1;
		B.SentSeconds = .1 + (Packet * 2 + 1) / 120.0 + 5;
		for (int32 I = FMath::Max(0, Packet * 2 - 4); I <= Packet * 2 + 1; ++I)
		{
			auto P = Wrist(I);
			P.CaptureSeconds += 5;
			B.Samples[B.Count++] = P;
		}
		TestTrue(TEXT("Delayed packet accepted without retimestamping"),
				 A.Admit(B, C, true, B.SentSeconds - 5 + .18, B.SentSeconds - 5 + .18, .4) ==
					 EBallInputDisposition::Accepted);
	}
	TestEqual(TEXT("All 20 captures recovered from redundancy"), A.Hand(EBallHandSide::Left).Num(), 20);
	TestEqual(TEXT("Last pose retains mapped capture time"),
			  A.Hand(EBallHandSide::Left).View().Last().Pose.TimelineSeconds, .1 + 19.0 / 120, 1.e-12);
	TestTrue(TEXT("Late input is observable for future adjudication"),
			 A.Statistics.Late > 0 && A.EarliestDirtyTimeline <= .1 + 1.e-12);
	TestEqual(TEXT("No late strike invented at receipt time"), Frame(A, .5).ColliderCount, 0);
	auto F = Frame(A);
	FBallContact H;
	TestTrue(TEXT("Historical wrist trajectory remains reconstructible"),
			 F.ColliderCount == 1 && FBallHandCollisionSolver::Query(
										 F, F.Colliders[0], PalmTests::Query(FVector(5, 0, 0), FVector(5, 0, 0), .5),
										 H) == EBallQueryResult::Contact);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetworkMissingGap,
								 "FourSquareVR.Ball.Network.Timing.MissingSamplesDoNotInventRotation",
								 NetworkPhysicsTestFlags)
bool FNetworkMissingGap::RunTest(const FString &)
{
	FBallPoseAdmission A;
	A.Initialize(Profile());
	for (int32 I = 0; I < 6; ++I)
	{
		auto P = Wrist(I);
		P.ControllerOrientation = FQuat::Identity;
		if (I >= 3)
			P.CaptureSeconds = P.TimelineSeconds = .2 + (I - 3) / 120.0;
		Feed(A, P);
	}
	TestEqual(TEXT("Unobserved gap beyond extrapolation is absent"), Frame(A, .17).ColliderCount, 0);
	TestEqual(TEXT("Continuous post-gap samples resume"), Frame(A, .2 + 1.0 / 120).ColliderCount, 1);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetworkRateError,
								 "FourSquareVR.Ball.Network.Physics.QuantizedTrajectoriesAt72To120Hz",
								 NetworkPhysicsTestFlags)
bool FNetworkRateError::RunTest(const FString &)
{
	FVector Base;
	bool First = true;
	for (int32 Rate : {72, 90, 120})
	{
		auto P = Profile();
		P.Left.BoxDimensions = FVector(8, 12, 2);
		P.Left.RoundRadius = .5;
		P.Transport.MaximumRPCsPerSecond = 240;
		FBallPoseAdmission A;
		A.Initialize(P);
		for (int32 I = 0; I <= Rate + 2; ++I)
		{
			const double T = double(I) / Rate;
			auto O = PalmTests::Pose(.1 + T, FVector(0, 0, -10 + 40 * T), FQuat(FVector::UpVector, 2 * T), I + 1);
			TestTrue(TEXT("Tracking rate captured without invented time"), Feed(A, O));
		}
		auto S = BallTests::State();
		FBallStepOutput O;
		int32 Events = 0;
		for (int32 I = 0; I < 120; ++I)
		{
			auto F = Frame(A, .1 + I / 120.0);
			F.Frame = I;
			if (PalmTests::Step(F, S, PalmTests::Config(), O) != EBallStepResult::Success)
			{
				AddError(TEXT("Network fixed step failed"));
				return false;
			}
			Events += O.EventCount;
		}
		TestEqual(TEXT("One physical strike"), Events, 1);
		TestEqual(TEXT("Analytic outgoing speed within quantization tolerance"), S.LinearVelocity.Z, 72.0, .2);
		if (First)
		{
			Base = S.Position;
			First = false;
		}
		else
			TestTrue(TEXT("Equivalent fixed-step endpoint within .05 cm"), S.Position.Equals(Base, .05));
		AddInfo(FString::Printf(TEXT("NET_RATE TrackingHz=%d EndZ=%.9f OutV=%.9f"), Rate, S.Position.Z,
								S.LinearVelocity.Z));
	}
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetworkFullBoundaries,
								 "FourSquareVR.Ball.Network.Snapshots.ConfigurationFinalityAndResetBoundaries",
								 NetworkPhysicsTestFlags)
bool FNetworkFullBoundaries::RunTest(const FString &)
{
	FBallNetworkSnapshot S;
	S.State = BallTests::RichState();
	S.Physics.Version = S.State.ConfigVersion;
	S.History.RevisionWindowSeconds = .25;
	S.Timebase = 2;
	S.bPaused = true;
	FBallWirePacket W;
	FBallNetworkSnapshot Q;
	TestTrue(TEXT("Full rich checkpoint encoded"), FBallNetworkCodec::WriteSnapshot(S, W));
	TestTrue(TEXT("Full rich checkpoint decoded"), FBallNetworkCodec::ReadSnapshot(W, Q));
	TestTrue(TEXT("Every collision/force/restoration field preserved"), BallTests::ExactlyEqual(S.State, Q.State));
	TestEqual(TEXT("250ms remains explicit policy"), Q.History.RevisionWindowSeconds, .25);
	Q.GeometryVersion = 1;
	TestFalse(TEXT("Incompatible geometry refused"), FBallNetworkCodec::WriteSnapshot(Q, W));
	Q = S;
	Q.Physics.Version++;
	TestFalse(TEXT("Inconsistent config refused"), FBallNetworkCodec::WriteSnapshot(Q, W));
	FBallAuthoritativeView V;
	V.Accept(S);
	for (uint32 Seq = 2; Seq < 32; ++Seq)
	{
		if (Seq % 3 == 0)
			continue;
		S.Sequence = Seq;
		++S.State.SimulationFrame;
		TestTrue(TEXT("Self-contained loss recovery"), V.Accept(S));
	}
	++S.Sequence;
	++S.Timebase;
	auto ResetIdentity = S.State.Identity;
	++ResetIdentity.Generation;
	S.State = BallTests::State();
	S.State.Identity = ResetIdentity;
	S.State.ConfigVersion = S.Physics.Version;
	TestTrue(TEXT("Explicit reset/timebase resynchronizes"), V.Accept(S));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FNetworkAdmissionCost,
								 "FourSquareVR.Ball.Network.Performance.BoundedAdmissionAndReconstruction",
								 NetworkPhysicsTestFlags)
bool FNetworkAdmissionCost::RunTest(const FString &)
{
	auto P = Profile();
	P.Transport.MaximumRPCsPerSecond = 240;
	FBallPoseAdmission A;
	A.Initialize(P);
	const SIZE_T Bytes = A.GetAllocatedBytes();
	const double Start = FPlatformTime::Seconds();
	for (int32 I = 0; I < 2000; ++I)
	{
		auto O = PalmTests::Pose(.1 + I / 120.0, FVector::ZeroVector, FQuat(FVector::UpVector, I * .3), I + 1);
		if (!Feed(A, O))
		{
			AddError(TEXT("Steady trajectory rejected"));
			return false;
		}
		auto F = Frame(A, O.CaptureSeconds - 1.0 / 120);
		if (I > 2 && F.ColliderCount != 1)
		{
			AddError(TEXT("Missing reconstructed collider"));
			return false;
		}
	}
	const double Us = (FPlatformTime::Seconds() - Start) * 1.e6 / 2000;
	TestEqual(TEXT("Capacity fixed under sustained input"), A.GetAllocatedBytes(), Bytes);
	TestEqual(TEXT("Only bounded recent captures retained"), A.Hand(EBallHandSide::Left).Num(), 256);
	AddInfo(FString::Printf(
		TEXT("NET_ADMISSION CodecValidateReconstructUs=%.3f DecodeExcludedAdmissionUs=%.3f HistoryBytes=%llu"), Us,
		A.Statistics.AdmissionMicroseconds / 2000, uint64(Bytes)));
	return true;
}
#endif
