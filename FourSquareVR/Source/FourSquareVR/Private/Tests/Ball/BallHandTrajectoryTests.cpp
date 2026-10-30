#include "Misc/AutomationTest.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "BallTestFixtures.h"
#include "Ball/BallHandPoseHistory.h"
#include "Ball/BallHandCollisionSolver.h"
#include "Ball/BallSimulationClock.h"
#include "Ball/HistoricalBallCollisionWorld.h"
#include "Ball/RecordedBallCollisionWorld.h"
#include "Ball/BallSimulator.h"
#include "Misc/Crc.h"
#include <limits>
namespace
{
constexpr auto Flags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;
FBallMotionFrame Frame()
{
	FBallMotionFrame F;
	F.Epoch = BallTests::State().Identity.Epoch;
	return F;
}
FBallHandObservation Pose(double Time, FVector Position, uint32 Sequence)
{
	FBallHandObservation P;
	P.CaptureSeconds = P.TimelineSeconds = Time;
	P.ControllerPosition = Position;
	P.HeadPosition = Position + FVector(0, 0, 50);
	P.Sequence = Sequence;
	return P;
}
FBallPhysicsConfig Config()
{
	FBallPhysicsConfig C;
	C.Gravity = FVector::ZeroVector;
	C.LinearDrag = C.AngularDrag = C.SurfaceFriction = C.SpinFriction = C.RollingResistance = 0;
	return C;
}
} // namespace
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandInvalidPose, "FourSquareVR.Ball.HandTrajectory.InvalidStaleAndFastSamples", Flags)
bool FHandInvalidPose::RunTest(const FString &)
{
	FBallHandPoseHistory H;
	H.Initialize(4);
	FBallHandTrackingSettings T;
	H.Add(Pose(0, FVector::ZeroVector, 1), T);
	H.Add(Pose(.01, FVector(1, 0, 0), 2), T);
	auto Bad = Pose(.02, FVector(2, 0, 0), 3);
	Bad.ControllerPosition.X = std::numeric_limits<double>::quiet_NaN();
	TestTrue(TEXT("NaN refused"), H.Add(Bad, T) == EBallTrackingRejection::InvalidPose);
	auto F = Frame();
	EBallTrackingRejection R;
	TestTrue(TEXT("Invalid pose cannot extend last valid trajectory"),
			 H.BuildMotion(.02, {}, T, 1, EBallHandSide::Left, F, R) && F.ColliderCount == 0);
	H.Add(Pose(.03, FVector(3, 0, 0), 4), T);
	H.Add(Pose(.04, FVector(4, 0, 0), 5), T);
	TestTrue(TEXT("Sample gap rejected"), H.Add(Pose(.2, FVector(20, 0, 0), 6), T) == EBallTrackingRejection::Stale);
	H.Add(Pose(.21, FVector(21, 0, 0), 7), T);
	auto Far = Pose(.22, FVector(22, 0, 0), 8);
	Far.HeadPosition = FVector(1000, 0, 0);
	TestTrue(TEXT("Unrealistic reach rejected"), H.Add(Far, T) == EBallTrackingRejection::ExcessiveReach);
	auto InvalidQ = Pose(.23, FVector(23, 0, 0), 9);
	InvalidQ.ControllerOrientation = FQuat(0, 0, 0, 0);
	TestTrue(TEXT("Non-unit quaternion rejected"), H.Add(InvalidQ, T) == EBallTrackingRejection::InvalidPose);
	H.Add(Pose(.24, FVector::ZeroVector, 10), T);
	auto Spin = Pose(.25, FVector::ZeroVector, 11);
	Spin.ControllerOrientation = FQuat(FVector::UpVector, PI);
	TestTrue(TEXT("Excessive angular speed rejected"), H.Add(Spin, T) == EBallTrackingRejection::ExcessiveVelocity);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandExtrapolation, "FourSquareVR.Ball.HandTrajectory.BoundedExtrapolationAndStorage",
								 Flags)
bool FHandExtrapolation::RunTest(const FString &)
{
	FBallHandPoseHistory H;
	H.Initialize(8);
	FBallHandTrackingSettings T;
	for (int32 I = 0; I < 3; ++I)
	{
		H.Add(Pose(I * .01, FVector(I, 0, 0), I + 1), T);
	}
	auto F = Frame();
	EBallTrackingRejection R;
	TestTrue(TEXT("Bounded extrapolation"), H.BuildMotion(.025, {}, T, 1, EBallHandSide::Left, F, R));
	TestEqual(TEXT("Extrapolated trajectory available"), F.ColliderCount, 1);
	TestEqual(TEXT("Timestamp-derived endpoint"), F.Segments[0].End.X, 100 * (.025 + 1.0 / 120), 1.e-9);
	F = Frame();
	H.BuildMotion(.033333333333, {}, T, 1, EBallHandSide::Left, F, R);
	TestTrue(TEXT("Extrapolation expires at an exact intra-step lifetime boundary"),
			 F.ColliderCount == 1 && F.Segments[0].EndFraction < 1);
	F = Frame();
	H.BuildMotion(.04, {}, T, 1, EBallHandSide::Left, F, R);
	TestEqual(TEXT("Stale hand disappears after bound"), F.ColliderCount, 0);
	const SIZE_T Bytes = H.GetAllocatedBytes();
	for (int32 I = 3; I < 1000; ++I)
	{
		H.Add(Pose(I * .01, FVector(I, 0, 0), I + 1), T);
	}
	TestEqual(TEXT("Raw observation ring stays bounded"), H.GetAllocatedBytes(), Bytes);
	TestTrue(TEXT("Latest sample remains continuous"), H.GetLatest()->bCollisionReady);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandOffsetCurve,
								 "FourSquareVR.Ball.HandTrajectory.RotatingPalmOffsetIsNotControllerVelocity", Flags)
bool FHandOffsetCurve::RunTest(const FString &)
{
	FBallHandPoseHistory H;
	H.Initialize();
	FBallHandTrackingSettings T;
	T.MaximumAngularSpeed = 1000;
	H.Add(Pose(0, FVector::ZeroVector, 1), T);
	H.Add(Pose(.01, FVector::ZeroVector, 2), T);
	auto P = Pose(.02, FVector::ZeroVector, 3);
	P.ControllerOrientation = FQuat(FVector::UpVector, PI / 2);
	H.Add(P, T);
	FBallPalmSettings Palm;
	Palm.CenterOffset = FVector(10, 0, 0);
	Palm.MaximumCurveError = .04;
	auto F = Frame();
	EBallTrackingRejection R;
	TestTrue(TEXT("Curve reconstruction"), H.BuildMotion(.01, Palm, T, 1, EBallHandSide::Left, F, R));
	TestEqual(TEXT("Conservative arc subdivisions"), F.SegmentCount, 8);
	TestTrue(TEXT("Controller itself stationary"), H.GetLatest()->LinearVelocity.IsZero());
	TestTrue(TEXT("Palm chord has real velocity"),
			 (F.Segments[0].End - F.Segments[0].Start).Size() /
					 ((F.Segments[0].EndFraction - F.Segments[0].StartFraction) * F.DeltaTime) >
				 1000);
	for (int32 I = 0; I < F.SegmentCount; ++I)
	{
		const auto &S = F.Segments[I];
		const double A = (S.StartFraction + S.EndFraction) * .5 * F.DeltaTime / .01 * PI / 2;
		const FVector True = FQuat(FVector::UpVector, A).RotateVector(Palm.CenterOffset);
		TestTrue(TEXT("Arc error within declared cm tolerance"),
				 (True - (S.Start + S.End) * .5).Size() <= Palm.MaximumCurveError + 1.e-9);
		TestTrue(TEXT("Offset rotation is not double-counted as surface spin"), S.SurfaceAngularVelocity.IsZero());
	}
	Palm.MaximumCurveError = 1.e-6;
	F = Frame();
	TestFalse(TEXT("Unresolvable curve rejected, never replaced by coarse chord"),
			  H.BuildMotion(.01, Palm, T, 1, EBallHandSide::Left, F, R));
	TestTrue(TEXT("Bounded curve capacity explicit"), R == EBallTrackingRejection::Capacity && F.ColliderCount == 0);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandRenderClock,
								 "FourSquareVR.Ball.HandTrajectory.CausalTrackingAcrossRenderingRates", Flags)
bool FHandRenderClock::RunTest(const FString &)
{
	FVector Baseline;
	bool First = true;
	for (int32 RenderHz : {30, 60, 72, 90, 120, 240})
	{
		FBallHandPoseHistory P;
		P.Initialize();
		FBallHandTrackingSettings T;
		FBallPalmSettings Palm;
		Palm.Material.Friction = 0;
		FBallMotionHistory H;
		H.Initialize(256);
		FRecordedBallCollisionWorld Static;
		FHistoricalBallCollisionWorld W(Static, H);
		FBallSimulator Sim;
		FBallSimulationClock Clock;
		Clock.Initialize({}, EBallClockRole::Authority);
		auto S = BallTests::State();
		FBallStepOutput O;
		int32 NextSample = 0, Events = 0;
		double Now = 0;
		while (Now < 1.1 - 1.e-12)
		{
			const double Dt = FMath::Min(1.0 / RenderHz, 1.1 - Now);
			Now += Dt;
			// Independent 90 Hz observations arrive before this rendering tick; no future samples are visible.
			while (NextSample / 90.0 <= Now + 1.e-12)
			{
				double Time = NextSample / 90.0;
				P.Add(Pose(Time, FVector(-100 + 100 * Time, 0, 0), NextSample + 1), T);
				++NextSample;
			}
			if (!Clock.BeginRenderFrame(Dt))
			{
				AddError(TEXT("Unexpected clock overrun"));
				return false;
			}
			while (Clock.HasStepAvailable())
			{
				auto F = Frame();
				F.Frame = S.SimulationFrame;
				EBallTrackingRejection R;
				if (!P.BuildMotion(F.Frame / 120.0, Palm, T, 1, EBallHandSide::Left, F, R) || !H.Push(F))
				{
					AddError(TEXT("Causal trajectory failed"));
					return false;
				}
				auto Inputs = BallTests::Inputs(S);
				Inputs.MotionTimebase = Inputs.MotionRevision = 1;
				if (Sim.SimulateStep(S, Inputs, Config(), W, O) != EBallStepResult::Success)
				{
					AddError(TEXT("Causal simulation failed"));
					return false;
				}
				Clock.ConsumeCompletedStep();
				Events += O.EventCount;
			}
		}
		TestEqual(TEXT("Same number of fixed steps"), S.SimulationFrame, uint64(132));
		TestEqual(TEXT("One strike"), Events, 1);
		TestEqual(TEXT("Correct relative velocity"), S.LinearVelocity.X, 180.0, 1.e-6);
		if (First)
		{
			Baseline = S.Position;
			First = false;
		}
		else
		{
			TestTrue(TEXT("Rendering-independent result from equivalent observations"),
					 S.Position.Equals(Baseline, 1.e-6));
		}
	}
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandTangent, "FourSquareVR.Ball.HandTrajectory.ExactAndNearTangency", Flags)
bool FHandTangent::RunTest(const FString &)
{
	auto F = Frame();
	FBallColliderMotion C;
	C.Id = BallMotionLimits::HandId(1, EBallHandSide::Left, 1);
	FBallMotionSegment S;
	S.Start = S.End = FVector::ZeroVector;
	F.Add(C, MakeArrayView(&S, 1));
	FBallCollisionQuery Q;
	Q.Start = FVector(-20, 10, 0);
	Q.End = FVector(20, 10, 0);
	Q.Radius = 3;
	FBallContact Hit;
	TestFalse(TEXT("Exact tangent does not create repeated zero-time contact"),
			  FBallHandCollisionSolver::Sweep(F, F.Colliders[0], Q, Hit));
	Q.Start.Y = Q.End.Y = 9.999;
	TestTrue(TEXT("Near tangent has real closing contact"), FBallHandCollisionSolver::Sweep(F, F.Colliders[0], Q, Hit));
	const double Expected = (20 - FMath::Sqrt(100 - 9.999 * 9.999)) / 40;
	TestEqual(TEXT("Near-tangent root"), Hit.TimeOfImpact, Expected, 1.e-9);
	TestTrue(TEXT("Normal normalized"), Hit.Normal.IsUnit(1.e-9));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandBoxCCD, "FourSquareVR.Ball.HandTrajectory.TranslatingOrientedBoxAnalyticCCD",
								 Flags)
bool FHandBoxCCD::RunTest(const FString &)
{
	auto F = Frame();
	FBallColliderMotion C;
	C.Id = {BallMotionLimits::KinematicNamespace | 1, 1};
	C.Category = EBallContactCategory::Kinematic;
	C.Shape = EBallMotionShape::TranslatingBox;
	C.Orientation = FQuat(FVector::UpVector, PI / 4);
	C.HalfExtents = FVector(5);
	C.Material.bOverride = true;
	C.Material.Friction = 0;
	FBallMotionSegment M;
	M.Start = FVector(-20, 0, 0);
	F.Add(C, MakeArrayView(&M, 1));
	FBallMotionHistory H;
	H.Initialize(2);
	H.Push(F);
	FRecordedBallCollisionWorld Static;
	FHistoricalBallCollisionWorld W(Static, H);
	auto S = BallTests::State();
	auto Inputs = BallTests::Inputs(S);
	Inputs.MotionTimebase = Inputs.MotionRevision = 1;
	FBallSimulator Sim;
	FBallStepOutput O;
	TestTrue(TEXT("Oriented recorded box"), Sim.SimulateStep(S, Inputs, Config(), W, O) == EBallStepResult::Success);
	const double TOI = (20 - (5 * FMath::Sqrt(2.0) + 3)) / 20;
	TestEqual(TEXT("Rounded vertex TOI"), O.Events[0].Time.Fraction, TOI, 1.e-8);
	TestEqual(TEXT("Relative response"), S.LinearVelocity.X, 4320.0, 1.e-6);
	TestEqual(TEXT("Correct remaining drift"), S.Position.X, .02 + 36 * (1 - TOI), 1.e-6);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandCheckpointV3, "FourSquareVR.Ball.HandTrajectory.FullPairCheckpointAndLegacyV2",
								 Flags)
bool FHandCheckpointV3::RunTest(const FString &)
{
	auto S = BallTests::RichState();
	S.MotionPairCount = BallLimits::MotionPairs;
	for (int32 I = 0; I < S.MotionPairCount; ++I)
	{
		auto &P = S.MotionPairs[I];
		P.Collider = {BallMotionLimits::HandNamespace | uint64(I + 1), uint32(I + 1)};
		P.Episode = I + 4;
		P.EventCounterAfterImpact = I + 2;
		P.bArmed = (I % 2) != 0;
		P.bBlocked = !P.bArmed;
		P.BlockReason = P.bBlocked ? EBallMotionBlockReason::ConflictingContact : EBallMotionBlockReason::None;
	}
	TArray<uint8> Bytes;
	FBallState R;
	TestTrue(TEXT("All pair state persists"),
			 FBallStateCodec::Save(S, Bytes) && FBallStateCodec::Load(Bytes, R) && BallTests::ExactlyEqual(S, R));
	S = BallTests::RichState();
	FBallStateCodec::Save(S, Bytes);
	Bytes.SetNum(Bytes.Num() - 4);
	const uint16 Version = 2;
	const uint32 Size = Bytes.Num() - 14, CRC = FCrc::MemCrc32(Bytes.GetData() + 14, Size);
	FMemory::Memcpy(Bytes.GetData() + 4, &Version, 2);
	FMemory::Memcpy(Bytes.GetData() + 6, &Size, 4);
	FMemory::Memcpy(Bytes.GetData() + 10, &CRC, 4);
	TestTrue(TEXT("Actual schema 2 migrates with empty dynamic pairs"),
			 FBallStateCodec::Load(Bytes, R) && BallTests::ExactlyEqual(S, R));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandFilterConfiguration,
								 "FourSquareVR.Ball.HandTrajectory.FilterBoundAndImmutableShapeVersions", Flags)
bool FHandFilterConfiguration::RunTest(const FString &)
{
	FBallHandPoseHistory H;
	H.Initialize();
	FBallHandTrackingSettings T;
	H.Add(Pose(0, FVector::ZeroVector, 1), T);
	H.Add(Pose(.01, FVector(1, 0, 0), 2), T);
	H.Add(Pose(.02, FVector(3, 0, 0), 3), T);
	const auto *S = H.GetLatest();
	TestTrue(TEXT("Extrapolation filter contributes bounded additional error"),
			 (S->ExtrapolationVelocity - S->LinearVelocity).Size() * T.MaximumExtrapolation <=
				 T.MaximumExtrapolationFilterError + 1.e-9);
	auto F = Frame();
	EBallTrackingRejection R;
	H.BuildMotion(.02, {}, T, 1, EBallHandSide::Left, F, R);
	TestTrue(TEXT("Recorded extrapolated movement and response velocity agree"),
			 (F.Segments[0].End - F.Segments[0].Start).Equals(S->ExtrapolationVelocity * F.DeltaTime, 1.e-9));
	FBallPalmSettings NewPalm;
	NewPalm.Radius = 30;
	F = Frame();
	H.BuildMotion(.01, NewPalm, T, 1, EBallHandSide::Left, F, R, 2);
	TestEqual(TEXT("New radius cannot reinterpret old samples"), F.ColliderCount, 0);
	auto P = Pose(.03, FVector(4, 0, 0), 4);
	P.ConfigurationVersion = 2;
	H.Add(P, T);
	P = Pose(.04, FVector(5, 0, 0), 5);
	P.ConfigurationVersion = 2;
	H.Add(P, T);
	F = Frame();
	H.BuildMotion(.04, NewPalm, T, 1, EBallHandSide::Left, F, R, 2);
	TestTrue(TEXT("New profile requires fresh generation"), F.ColliderCount == 1 && F.Colliders[0].Id.Generation > 1 &&
																F.Colliders[0].Radius == 30 &&
																F.Colliders[0].ConfigurationVersion == 2);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandEpochBoundary, "FourSquareVR.Ball.HandTrajectory.CrossWorldEpochCannotReplay",
								 Flags)
bool FHandEpochBoundary::RunTest(const FString &)
{
	auto F = Frame();
	F.Epoch = FGuid(9, 8, 7, 6);
	FBallMotionHistory H;
	H.Initialize(2);
	H.Push(F);
	FRecordedBallCollisionWorld Static;
	FHistoricalBallCollisionWorld W(Static, H);
	auto S = BallTests::State();
	const auto Before = S;
	auto Inputs = BallTests::Inputs(S);
	Inputs.MotionTimebase = Inputs.MotionRevision = 1;
	FBallSimulator Sim;
	FBallStepOutput O;
	TestTrue(TEXT("Same numeric frame/version from another world rejected"),
			 Sim.SimulateStep(S, Inputs, Config(), W, O) == EBallStepResult::IncompatibleEnvironment);
	TestTrue(TEXT("Whole state unchanged"), BallTests::ExactlyEqual(S, Before));
	TestEqual(TEXT("No speculative events escape"), O.EventCount, 0);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandContactCapacity,
								 "FourSquareVR.Ball.HandTrajectory.ManifoldCapacityFailsAtomically", Flags)
bool FHandContactCapacity::RunTest(const FString &)
{
	auto F = Frame();
	for (int32 I = 0; I < 12; ++I)
	{
		FBallColliderMotion C;
		C.Id = I < 8 ? BallMotionLimits::HandId(I / 2 + 1, EBallHandSide(I % 2), 1)
					 : FBallColliderId{BallMotionLimits::KinematicNamespace | uint64(I - 7), 1};
		C.Category = I < 8 ? EBallContactCategory::Hand : EBallContactCategory::Kinematic;
		C.Material.bOverride = true;
		C.Material.Friction = 0;
		FBallMotionSegment M;
		M.Start = FVector(-20, 0, 0);
		TestTrue(TEXT("Supported total collider capacity"), F.Add(C, MakeArrayView(&M, 1)));
	}
	FBallMotionHistory H;
	H.Initialize(2);
	H.Push(F);
	FRecordedBallCollisionWorld Static;
	FHistoricalBallCollisionWorld W(Static, H);
	auto S = BallTests::State();
	const auto Before = S;
	auto Inputs = BallTests::Inputs(S);
	Inputs.MotionTimebase = Inputs.MotionRevision = 1;
	FBallSimulator Sim;
	FBallStepOutput O;
	TestTrue(TEXT("Overfull simultaneous manifold is explicit"),
			 Sim.SimulateStep(S, Inputs, Config(), W, O) == EBallStepResult::ContactCapacity);
	TestTrue(TEXT("No partial positional/velocity/counter changes"), BallTests::ExactlyEqual(S, Before));
	TestTrue(TEXT("No partial events"), O.EventCount == 0 && O.ContactCount == 0);
	return true;
}
#endif
