#include "Misc/AutomationTest.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "BallPalmTestFixtures.h"
#include "Ball/BallHandPoseHistory.h"
#include "Ball/BallReplayRunner.h"
#include "Ball/BallStateCodec.h"
#include "Ball/BallCollisionSolver.h"
#include "Ball/BallSimulationClock.h"
#include <limits>
namespace
{
constexpr auto PalmHistoryFlags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmRecordedCalibration,
								 "FourSquareVR.Ball.Palm.History.RecordedOffsetArcAndQuaternionDerivative",
								 PalmHistoryFlags)
bool FPalmRecordedCalibration::RunTest(const FString &)
{
	FBallHandPoseHistory H;
	H.Initialize();
	FBallHandTrackingSettings T;
	T.MaximumAngularSpeed = 200;
	FBallPalmSettings P;
	P.CenterOffset = FVector(10, 0, 0);
	P.RotationOffset = FRotator(10, 30, 20);
	for (int32 I = 0; I < 3; ++I)
		H.Add(PalmTests::Pose(I * .01, FVector::ZeroVector, FQuat(FVector::UpVector, I * .4), I + 1), T);
	FBallMotionFrame F;
	F.Epoch = BallTests::State().Identity.Epoch;
	EBallTrackingRejection R;
	TestTrue(TEXT("Recorded"), H.BuildMotion(.01, P, T, 1, EBallHandSide::Left, F, R));
	if (!TestEqual(TEXT("Exact arc needs one segment"), F.SegmentCount, 1))
		return false;
	const auto &C = F.Colliders[0];
	const auto &S = F.Segments[0];
	const double Fraction = .37, Angle = .4 + 40 * Fraction * F.DeltaTime;
	const FQuat Q(FVector::UpVector, Angle);
	TestTrue(TEXT("World center follows exact offset arc"),
			 FBallPalmGeometry::CenterAt(C, S, Fraction).Equals(Q.RotateVector(P.CenterOffset), 1.e-8));
	TestTrue(TEXT("Calibrated orientation recorded"),
			 FBallPalmGeometry::OrientationAt(S, Fraction).Equals(Q * P.RotationOffset.Quaternion(), 1.e-8));
	const FVector Center = FBallPalmGeometry::CenterAt(C, S, Fraction), Point = Center + FVector(0, 1, 0);
	TestTrue(TEXT("No double counted offset velocity"),
			 FBallPalmGeometry::SurfaceVelocity(C, S, Fraction, F.DeltaTime, Point)
				 .Equals(FVector::CrossProduct(FVector(0, 0, 40), Point), 1.e-7));
	TestTrue(TEXT("Calibration identity retained"),
			 C.CalibrationRotation.Equals(P.RotationOffset.Quaternion(), 1.e-10));
	const double E = 1.e-5;
	const FVector Numeric =
		(FBallPalmGeometry::CenterAt(C, S, Fraction + E) - FBallPalmGeometry::CenterAt(C, S, Fraction - E)) /
		(2 * E * F.DeltaTime);
	TestTrue(TEXT("Contact derivative matches geometry"),
			 Numeric.Equals(FBallPalmGeometry::SurfaceVelocity(C, S, Fraction, F.DeltaTime, Center), 1.e-5));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmAngularSafety,
								 "FourSquareVR.Ball.Palm.History.QuaternionSignsAmbiguityLossAndDiscontinuities",
								 PalmHistoryFlags)
bool FPalmAngularSafety::RunTest(const FString &)
{
	FBallHandPoseHistory H;
	H.Initialize();
	FBallHandTrackingSettings T;
	T.MaximumAngularSpeed = 1000;
	H.Add(PalmTests::Pose(0, FVector::ZeroVector, FQuat::Identity, 1), T);
	const FQuat Q(FVector::UpVector, .1);
	TestTrue(TEXT("Sign-equivalent quaternion continuous"),
			 H.Add(PalmTests::Pose(.01, FVector::ZeroVector, Q * -1, 2), T) == EBallTrackingRejection::None);
	TestEqual(TEXT("Physical angular speed, not 2pi alias"), H.GetLatest()->AngularVelocity.Z, 10.0, 1.e-9);
	TestTrue(TEXT("Large ambiguous rotation rejected"),
			 H.Add(PalmTests::Pose(.02, FVector::ZeroVector, FQuat(FVector::UpVector, 3.1), 3), T) ==
				 EBallTrackingRejection::UnsupportedRotation);
	TestFalse(TEXT("Invalid rotation cannot collide"), H.GetLatest()->bCollisionReady);
	H.Add(PalmTests::Pose(.03, FVector::ZeroVector, Q, 4), T);
	TestFalse(TEXT("Reacquisition warmup"), H.GetLatest()->bCollisionReady);
	H.Add(PalmTests::Pose(.04, FVector::ZeroVector, Q, 5), T);
	TestTrue(TEXT("Coherent continuity resumes"), H.GetLatest()->bCollisionReady);
	const uint32 Gen = H.GetLatest()->Generation;
	auto Bad = PalmTests::Pose(.05, FVector::ZeroVector, Q, 6);
	Bad.ControllerOrientation.X = std::numeric_limits<double>::infinity();
	TestTrue(TEXT("Infinite quaternion rejected"), H.Add(Bad, T) == EBallTrackingRejection::InvalidPose);
	H.Add(PalmTests::Pose(.06, FVector::ZeroVector, Q, 7), T);
	TestTrue(TEXT("Discontinuity is a new lifetime"),
			 H.GetLatest()->Generation > Gen && !H.GetLatest()->bCollisionReady);
	Bad = PalmTests::Pose(.07, FVector::ZeroVector, Q, 8);
	Bad.bTracked = false;
	TestTrue(TEXT("Tracking loss"), H.Add(Bad, T) == EBallTrackingRejection::Untracked);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmSmallRotations,
								 "FourSquareVR.Ball.Palm.History.SmallAnglesAndOrientationExtrapolation",
								 PalmHistoryFlags)
bool FPalmSmallRotations::RunTest(const FString &)
{
	const FQuat A = FQuat::Identity, B(FVector::UpVector, 1.e-8);
	TestEqual(TEXT("Tiny angular delta retains physical velocity"), FBallPalmGeometry::RotationVector(A, B).Z, 1.e-8,
			  1.e-15);
	TestEqual(TEXT("Small-angle interpolation is constant-speed"),
			  FBallPalmGeometry::RotationVector(A, FBallPalmGeometry::InterpolateOrientation(A, B, .3)).Z, 3.e-9,
			  1.e-15);
	FBallHandPoseHistory H;
	H.Initialize();
	FBallHandTrackingSettings T;
	for (int32 I = 0; I < 3; ++I)
		H.Add(PalmTests::Pose(I * .01, FVector::ZeroVector, FQuat(FVector::UpVector, I * .1), I + 1), T);
	FBallMotionFrame F;
	F.Epoch = BallTests::State().Identity.Epoch;
	EBallTrackingRejection R;
	H.BuildMotion(.025, {}, T, 1, EBallHandSide::Left, F, R);
	if (!TestEqual(TEXT("Extrapolated orientation available"), F.ColliderCount, 1))
		return false;
	TestTrue(TEXT("Bounded orientation at fixed boundary"),
			 F.Segments[0].StartOrientation.Equals(FQuat(FVector::UpVector, .25), 1.e-9) &&
				 F.Segments[0].EndOrientation.Equals(FQuat(FVector::UpVector, 10 * (.025 + 1.0 / 120)), 1.e-9));
	F.GeometryVersion = 1;
	TestFalse(TEXT("Legacy geometry epoch cannot replay as rounded motion"), F.IsValid());
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmMaterialOverride,
								 "FourSquareVR.Ball.Palm.Regression.MaterialOverrideSwitchHonored", PalmHistoryFlags)
bool FPalmMaterialOverride::RunTest(const FString &)
{
	for (EBallPalmShape Shape : {EBallPalmShape::Sphere, EBallPalmShape::RoundedBox})
		for (bool Override : {false, true})
		{
			FBallHandPoseHistory H;
			H.Initialize();
			FBallHandTrackingSettings T;
			for (int32 I = 0; I < 3; ++I)
				H.Add(PalmTests::Pose(I * .01, FVector(0, 0, -5 + I), FQuat::Identity, I + 1), T);
			FBallPalmSettings P;
			P.Shape = Shape;
			P.Material.bOverride = Override;
			P.Material.Restitution = .2;
			FBallMotionFrame F;
			F.Epoch = BallTests::State().Identity.Epoch;
			EBallTrackingRejection R;
			H.BuildMotion(.02, P, T, 1, EBallHandSide::Left, F, R);
			if (!TestEqual(TEXT("Motion"), F.ColliderCount, 1))
				return false;
			TestEqual(TEXT("Exposed override copied faithfully"), F.Colliders[0].Material.bOverride, Override);
			FBallContactBatch B;
			B.Count = 1;
			auto &K = B.Contacts[0];
			K.Collider = F.Colliders[0].Id;
			K.Material = F.Colliders[0].Material;
			K.Normal = FVector::UpVector;
			auto S = BallTests::State();
			S.LinearVelocity.Z = -100;
			auto C = PalmTests::Config();
			FBallStepOutput O;
			TestTrue(TEXT("Response"), FBallCollisionSolver::Resolve(S, B, C, .5, O) == EBallStepResult::Success);
			TestEqual(TEXT("Off inherits ball e, on combines actual palm e"), S.LinearVelocity.Z,
					  Override ? 40.0 : 80.0, 1.e-8);
		}
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmCaps, "FourSquareVR.Ball.Palm.Regression.EndpointSpeedCapsAndEventConsistency",
								 PalmHistoryFlags)
bool FPalmCaps::RunTest(const FString &)
{
	// At t=1 a translating rounded box hits; no remaining force Kick can hide the regression.
	auto F = PalmTests::Frame(FVector(0, 0, -7), FVector(0, 0, -2));
	auto S = BallTests::State();
	auto C = PalmTests::Config();
	C.MaximumLinearSpeed = 800;
	FBallStepOutput O;
	TestTrue(TEXT("Endpoint capped strike"), PalmTests::Step(F, S, C, O) == EBallStepResult::Success);
	TestEqual(TEXT("Speed cap applies at endpoint"), S.LinearVelocity.Z, 800.0, 1.e-7);
	if (!TestEqual(TEXT("One event"), O.EventCount, 1))
		return false;
	TestEqual(TEXT("Published event agrees with capped state"), O.Events[0].OutgoingVelocity.Z, 800.0, 1.e-7);
	F = PalmTests::Frame();
	auto &M = F.Colliders[0];
	M.Material.Friction = .8;
	S = BallTests::State();
	S.Position.Z = 5;
	S.LinearVelocity = FVector(300, 0, -1200);
	C = PalmTests::Config();
	C.SurfaceFriction = .8;
	C.MaximumAngularSpeed = 50;
	TestTrue(TEXT("Angular contact cap"), PalmTests::Step(F, S, C, O) == EBallStepResult::Success);
	TestEqual(TEXT("Friction spin bounded"), S.AngularVelocity.Size(), 50.0, 1.e-8);
	TestEqual(TEXT("No restrictive caps introduced by default"), PalmTests::Config().MaximumLinearSpeed, 0.0);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmImpossibleCap,
								 "FourSquareVR.Ball.Palm.Regression.InfeasibleSpeedCapFailsAtomically",
								 PalmHistoryFlags)
bool FPalmImpossibleCap::RunTest(const FString &)
{
	auto F = PalmTests::Frame(FVector(0, 0, -7), FVector(0, 0, -2));
	auto S = BallTests::State();
	const auto Before = S;
	auto C = PalmTests::Config();
	C.MaximumLinearSpeed = 100;
	FBallStepOutput O;
	TestTrue(TEXT("A cap below mandatory surface speed cannot be safely published"),
			 PalmTests::Step(F, S, C, O) == EBallStepResult::CollisionLimit);
	TestTrue(TEXT("Original state retained"), BallTests::ExactlyEqual(S, Before));
	TestEqual(TEXT("No event"), O.EventCount, 0);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmReplay,
								 "FourSquareVR.Ball.Palm.History.RecordedRotationsRestoreCompleteStateAndEvents",
								 PalmHistoryFlags)
bool FPalmReplay::RunTest(const FString &)
{
	FBallMotionHistory H;
	H.Initialize(256);
	FBallHistoryBuffer States;
	auto S = BallTests::State();
	S.Position = FVector(5, 0, 0);
	const auto Initial = S;
	States.Initialize(256, S.Identity);
	auto C = PalmTests::Config();
	C.Radius = .5;
	FRecordedBallCollisionWorld Static;
	FHistoricalBallCollisionWorld W(Static, H);
	FBallSimulator Sim;
	FBallStepOutput O;
	TArray<FBallCollisionEvent> Events;
	for (int32 I = 0; I < 40; ++I)
	{
		auto F = PalmTests::Wrist();
		F.Frame = I;
		F.Segments[0].StartOrientation = FQuat(FVector::UpVector, -.3 + I * .6);
		F.Segments[0].EndOrientation = FQuat(FVector::UpVector, .3 + I * .6);
		auto Inputs = BallTests::Inputs(S);
		Inputs.MotionTimebase = Inputs.MotionRevision = 1;
		if (!H.Push(F) || States.Push({S, Inputs}) != EBallHistoryWrite::Stored ||
			Sim.SimulateStep(S, Inputs, C, W, O) != EBallStepResult::Success)
		{
			AddError(FString::Printf(TEXT("Historical step %d failed %d"), I, int32(O.Result)));
			return false;
		}
		for (int32 J = 0; J < O.EventCount; ++J)
			Events.Add(O.Events[J]);
	}
	TestTrue(TEXT("Actual rotational event"), Events.Num() > 0);
	FBallReplayRunner Runner;
	FBallReplayReport Report;
	FBallState Replayed;
	TestTrue(TEXT("Bounded thirty-frame replay"),
			 Runner.Replay(States, S.Identity, 10, 40, C, {}, W, Replayed, Report) == EBallReplayResult::Success);
	TestTrue(TEXT("Every dynamic/contact field restored"), BallTests::ExactlyEqual(S, Replayed));
	Replayed = Initial;
	int32 Index = 0;
	for (int32 I = 0; I < 40; ++I)
	{
		auto Inputs = BallTests::Inputs(Replayed);
		Inputs.MotionTimebase = Inputs.MotionRevision = 1;
		if (Sim.SimulateStep(Replayed, Inputs, C, W, O) != EBallStepResult::Success)
			return false;
		for (int32 J = 0; J < O.EventCount; ++J)
		{
			if (Index >= Events.Num())
			{
				AddError(TEXT("Duplicate replay event"));
				return false;
			}
			const auto &A = Events[Index++], &B = O.Events[J];
			TestTrue(TEXT("Stable ID, timing, normal and outcome"),
					 A.Ordinal == B.Ordinal && A.Collider == B.Collider && A.Episode == B.Episode &&
						 A.FeatureId == B.FeatureId && A.Time.Frame == B.Time.Frame &&
						 A.Time.Fraction == B.Time.Fraction && A.Normal == B.Normal &&
						 A.OutgoingVelocity == B.OutgoingVelocity);
		}
	}
	TestEqual(TEXT("Identical event count"), Index, Events.Num());
	TestTrue(TEXT("Final state equal"), BallTests::ExactlyEqual(S, Replayed));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmConfigEpoch,
								 "FourSquareVR.Ball.Palm.History.CalibrationChangesCannotReinterpretPastMotion",
								 PalmHistoryFlags)
bool FPalmConfigEpoch::RunTest(const FString &)
{
	FBallHandPoseHistory H;
	H.Initialize();
	FBallHandTrackingSettings T;
	for (int32 I = 0; I < 3; ++I)
		H.Add(PalmTests::Pose(I * .01, FVector::ZeroVector, FQuat(FVector::UpVector, I * .1), I + 1), T);
	FBallPalmSettings P;
	FBallMotionFrame F;
	F.Epoch = BallTests::State().Identity.Epoch;
	EBallTrackingRejection R;
	H.BuildMotion(.02, P, T, 1, EBallHandSide::Left, F, R);
	const auto Frozen = F;
	P.BoxDimensions.X = 15;
	P.RotationOffset.Yaw = 60;
	P.CenterOffset.X = 5;
	F.ColliderCount = F.SegmentCount = 0;
	H.BuildMotion(.02, P, T, 1, EBallHandSide::Left, F, R, 2);
	TestEqual(TEXT("Old samples cannot acquire new geometry"), F.ColliderCount, 0);
	TestTrue(TEXT("Frozen accepted trajectory owns old dimensions and quaternion"),
			 Frozen.Colliders[0].HalfExtents.X == 4.5 && Frozen.Colliders[0].PivotToCenter.IsZero() &&
				 Frozen.Segments[0].StartOrientation.Equals(FQuat(FVector::UpVector, .2), 1.e-9));
	F = Frozen;
	F.Segments[0].EndOrientation = FQuat(FVector::UpVector, PI) * F.Segments[0].StartOrientation;
	TestFalse(TEXT("Ambiguous historical rotation rejected"), F.IsValid());
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmRateEquivalence,
								 "FourSquareVR.Ball.Palm.History.EquivalentPoseTrajectoriesAt72To240Hz",
								 PalmHistoryFlags)
bool FPalmRateEquivalence::RunTest(const FString &)
{
	FVector Baseline;
	bool First = true;
	for (int32 Rate : {72, 90, 120, 240})
	{
		FBallHandPoseHistory H;
		H.Initialize(512);
		FBallHandTrackingSettings T;
		FBallPalmSettings P;
		P.BoxDimensions = FVector(8, 12, 2);
		P.RoundRadius = .5;
		P.Material.Friction = 0;
		for (int32 I = 0; I <= Rate + 2; ++I)
		{
			const double Time = double(I) / Rate;
			H.Add(PalmTests::Pose(Time, FVector(0, 0, -10 + 40 * Time), FQuat(FVector::UpVector, 2 * Time), I + 1), T);
		}
		auto S = BallTests::State();
		FBallStepOutput O;
		int32 Events = 0;
		for (int32 I = 0; I < 120; ++I)
		{
			FBallMotionFrame F;
			F.Epoch = S.Identity.Epoch;
			F.Frame = I;
			EBallTrackingRejection R;
			if (!H.BuildMotion(I / 120.0, P, T, 1, EBallHandSide::Left, F, R) ||
				PalmTests::Step(F, S, PalmTests::Config(), O) != EBallStepResult::Success)
			{
				AddError(FString::Printf(TEXT("Rate %d frame %d failed %d"), Rate, I, int32(O.Result)));
				return false;
			}
			Events += O.EventCount;
		}
		TestEqual(TEXT("One eligible strike"), Events, 1);
		TestEqual(TEXT("Linear/angular equivalent normal response"), S.LinearVelocity.Z, 72.0, 2.e-5);
		if (First)
		{
			Baseline = S.Position;
			First = false;
		}
		else
			TestTrue(TEXT("Same fixed-step result"), S.Position.Equals(Baseline, 5.e-5));
	}
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmCausalRenderRates,
								 "FourSquareVR.Ball.Palm.History.CausalOrientationAcrossRenderRates", PalmHistoryFlags)
bool FPalmCausalRenderRates::RunTest(const FString &)
{
	FVector Baseline;
	bool First = true;
	for (int32 Rate : {30, 60, 72, 90, 120, 240})
	{
		FBallHandPoseHistory H;
		H.Initialize();
		FBallHandTrackingSettings Tracking;
		FBallPalmSettings P;
		P.BoxDimensions = FVector(8, 12, 2);
		P.RoundRadius = .5;
		P.Material.Friction = 0;
		FBallSimulationClock Clock;
		Clock.Initialize({}, EBallClockRole::Authority);
		auto S = BallTests::State();
		FBallStepOutput O;
		double Now = 0;
		int32 NextSample = 0, Events = 0;
		while (Now < 1.1 - 1.e-12)
		{
			const double Delta = FMath::Min(1.0 / Rate, 1.1 - Now);
			Now += Delta;
			while (NextSample / 90.0 <= Now + 1.e-12)
			{
				const double T = NextSample / 90.0;
				H.Add(PalmTests::Pose(T, FVector(0, 0, -10 + 40 * T), FQuat(FVector::UpVector, 2 * T), NextSample + 1),
					  Tracking);
				++NextSample;
			}
			if (!Clock.BeginRenderFrame(Delta))
			{
				AddError(TEXT("Unexpected render-clock overrun"));
				return false;
			}
			while (Clock.HasStepAvailable())
			{
				FBallMotionFrame F;
				F.Epoch = S.Identity.Epoch;
				F.Frame = S.SimulationFrame;
				EBallTrackingRejection R;
				if (!H.BuildMotion(F.Frame / 120.0, P, Tracking, 1, EBallHandSide::Left, F, R) ||
					PalmTests::Step(F, S, PalmTests::Config(), O) != EBallStepResult::Success)
				{
					AddError(FString::Printf(TEXT("Causal pose failure rate %d frame %llu"), Rate, S.SimulationFrame));
					return false;
				}
				Clock.ConsumeCompletedStep();
				Events += O.EventCount;
			}
		}
		TestEqual(TEXT("Same fixed step count"), S.SimulationFrame, uint64(132));
		TestEqual(TEXT("Exactly one strike"), Events, 1);
		TestEqual(TEXT("Correct normal response"), S.LinearVelocity.Z, 72.0, 2.e-5);
		if (First)
		{
			Baseline = S.Position;
			First = false;
		}
		else
			TestTrue(TEXT("Render rate does not change represented physics"), S.Position.Equals(Baseline, 5.e-5));
	}
	return true;
}
#endif
