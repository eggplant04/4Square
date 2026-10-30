#include "Misc/AutomationTest.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "BallTestFixtures.h"
#include "Ball/BallSimulator.h"
#include "Ball/BallHandPoseHistory.h"
#include "Ball/HistoricalBallCollisionWorld.h"
#include "Ball/RecordedBallCollisionWorld.h"
#include "Ball/BallReplayRunner.h"
#include "HAL/PlatformTime.h"

namespace
{
constexpr auto Flags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;
FBallPhysicsConfig Config()
{
	FBallPhysicsConfig C;
	C.Gravity = FVector::ZeroVector;
	C.LinearDrag = C.AngularDrag = C.SurfaceFriction = C.SpinFriction = C.RollingResistance = 0;
	return C;
}
void Palm(FBallMotionFrame &F, FVector A, FVector B, EBallHandSide Side = EBallHandSide::Left, double E = 0.8)
{
	FBallColliderMotion C;
	C.Id = BallMotionLimits::HandId(1, Side, 1);
	C.Participant = 1;
	C.Side = Side;
	C.OriginRevision = 1;
	C.Material.bOverride = true;
	C.Material.Restitution = E;
	C.Material.Friction = 0;
	FBallMotionSegment S;
	S.Start = A;
	S.End = B;
	S.FirstSample = 20;
	S.SecondSample = 21;
	check(F.Add(C, MakeArrayView(&S, 1)));
}
FBallStepInputs MotionInputs(const FBallState &S)
{
	auto I = BallTests::Inputs(S);
	I.MotionTimebase = I.MotionRevision = 1;
	return I;
}
EBallStepResult Step(FBallMotionFrame &F, FBallState &S, const FBallPhysicsConfig &C, FBallStepOutput &O,
					 const IBallCollisionWorld *Static = nullptr)
{
	F.Frame = S.SimulationFrame;
	FBallMotionHistory H;
	H.Initialize(2);
	H.Push(F);
	FRecordedBallCollisionWorld Empty;
	FHistoricalBallCollisionWorld W(Static ? *Static : Empty, H);
	FBallSimulator Sim;
	return Sim.SimulateStep(S, MotionInputs(S), C, W, O);
}
FBallHandObservation Pose(double T, FVector P, uint32 Seq)
{
	FBallHandObservation R;
	R.CaptureSeconds = R.TimelineSeconds = T;
	R.ControllerPosition = P;
	R.HeadPosition = P + FVector(0, 0, 50);
	R.Sequence = Seq;
	return R;
}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandStationaryBallTest, "FourSquareVR.Ball.Hand.MovingPalmStrikesStationaryBall",
								 Flags)
bool FHandStationaryBallTest::RunTest(const FString &)
{
	FBallMotionFrame F;
	F.Epoch = BallTests::State().Identity.Epoch;
	Palm(F, FVector(-20, 0, 0), FVector::ZeroVector);
	auto S = BallTests::State();
	FBallStepOutput O;
	TestTrue(TEXT("Actual simulation succeeds"), Step(F, S, Config(), O) == EBallStepResult::Success);
	TestEqual(TEXT("One collision event"), O.EventCount, 1);
	TestEqual(TEXT("TOI half-step"), O.Events[0].Time.Fraction, 0.5, 1.e-9);
	TestEqual(TEXT("Relative restitution: 2400*(1+.8)"), S.LinearVelocity.X, 4320.0, 1.e-6);
	TestEqual(TEXT("Remaining drift plus documented skin"), S.Position.X, 18.02, 1.e-7);
	TestEqual(TEXT("Historical sample provenance"), O.Events[0].SecondSample, uint32(21));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandStationaryPalmTest, "FourSquareVR.Ball.Hand.MovingBallStrikesStationaryPalm",
								 Flags)
bool FHandStationaryPalmTest::RunTest(const FString &)
{
	FBallMotionFrame F;
	F.Epoch = BallTests::State().Identity.Epoch;
	Palm(F, FVector(20, 0, 0), FVector(20, 0, 0));
	auto S = BallTests::State();
	S.LinearVelocity.X = 2400;
	FBallStepOutput O;
	TestTrue(TEXT("Simulation"), Step(F, S, Config(), O) == EBallStepResult::Success);
	TestEqual(TEXT("TOI"), O.Events[0].Time.Fraction, 0.5, 1.e-9);
	TestEqual(TEXT("Bounce speed"), S.LinearVelocity.X, -1920.0, 1.e-6);
	TestEqual(TEXT("Bounce remaining position"), S.Position.X, 1.98, 1.e-7);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandOpposingTest, "FourSquareVR.Ball.Hand.FastOpposingMotion", Flags)
bool FHandOpposingTest::RunTest(const FString &)
{
	FBallMotionFrame F;
	F.Epoch = BallTests::State().Identity.Epoch;
	Palm(F, FVector(20, 0, 0), FVector::ZeroVector);
	auto S = BallTests::State();
	S.Position.X = -20;
	S.LinearVelocity.X = 4800;
	FBallStepOutput O;
	TestTrue(TEXT("Simulation"), Step(F, S, Config(), O) == EBallStepResult::Success);
	TestEqual(TEXT("Relative TOI"), O.Events[0].Time.Fraction, 0.5, 1.e-9);
	TestEqual(TEXT("Relative speed response"), S.LinearVelocity.X, -8160.0, 1.e-6);
	TestEqual(TEXT("Final position"), S.Position.X, -34.02, 1.e-7);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandGlancingTest, "FourSquareVR.Ball.Hand.AnalyticGlancingStrike", Flags)
bool FHandGlancingTest::RunTest(const FString &)
{
	FBallMotionFrame F;
	F.Epoch = BallTests::State().Identity.Epoch;
	Palm(F, FVector::ZeroVector, FVector::ZeroVector);
	auto S = BallTests::State();
	S.Position = FVector(-20, 8, 0);
	S.LinearVelocity.X = 4800;
	FBallStepOutput O;
	TestTrue(TEXT("Simulation"), Step(F, S, Config(), O) == EBallStepResult::Success);
	TestEqual(TEXT("Rounded sphere TOI"), O.Events[0].Time.Fraction, 0.35, 1.e-9);
	TestTrue(TEXT("Correct normal"), O.Events[0].Normal.Equals(FVector(-0.6, 0.8, 0), 1.e-8));
	TestTrue(TEXT("Normal restitution with conserved tangential component"),
			 S.LinearVelocity.Equals(FVector(1689.6, 4147.2, 0), 1.e-6));
	TestTrue(TEXT("Passive collision dissipates energy"), S.LinearVelocity.SizeSquared() < FMath::Square(4800.0));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandTunnelingTest, "FourSquareVR.Ball.Hand.HighSpeedRelativeTunneling", Flags)
bool FHandTunnelingTest::RunTest(const FString &)
{
	FBallMotionFrame F;
	F.Epoch = BallTests::State().Identity.Epoch;
	Palm(F, FVector::ZeroVector, FVector::ZeroVector);
	auto S = BallTests::State();
	S.Position.X = -10000;
	S.LinearVelocity.X = 2400000;
	FBallStepOutput O;
	TestTrue(TEXT("Simulation"), Step(F, S, Config(), O) == EBallStepResult::Success);
	TestEqual(TEXT("Very thin temporal contact found"), O.EventCount, 1);
	TestEqual(TEXT("Analytic TOI"), O.Events[0].Time.Fraction, 9990.0 / 20000, 1.e-9);
	TestEqual(TEXT("Correct speed"), S.LinearVelocity.X, -1920000.0, 1.e-5);
	TestTrue(TEXT("Remains on incoming side"), S.Position.X < -10);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandBothTest, "FourSquareVR.Ball.Hand.BothPalmsSimultaneous", Flags)
bool FHandBothTest::RunTest(const FString &)
{
	FBallMotionFrame F;
	F.Epoch = BallTests::State().Identity.Epoch;
	Palm(F, FVector(-20, 0, 0), FVector::ZeroVector);
	Palm(F, FVector(0, -20, 0), FVector::ZeroVector, EBallHandSide::Right);
	auto S = BallTests::State();
	FBallStepOutput O;
	TestTrue(TEXT("Joint solve"), Step(F, S, Config(), O) == EBallStepResult::Success);
	TestEqual(TEXT("Two unique events"), O.EventCount, 2);
	TestTrue(TEXT("Stable left/right ordering"), O.Events[0].Collider.Value < O.Events[1].Collider.Value);
	TestTrue(TEXT("Both relative impulses"), S.LinearVelocity.Equals(FVector(4320, 4320, 0), 1.e-6));
	TestTrue(TEXT("Both remaining displacements"), S.Position.Equals(FVector(18.02, 18.02, 0), 1.e-7));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandConflictTest, "FourSquareVR.Ball.Hand.ConflictingPalmsHaveBoundedPolicy", Flags)
bool FHandConflictTest::RunTest(const FString &)
{
	FBallMotionFrame F;
	F.Epoch = BallTests::State().Identity.Epoch;
	Palm(F, FVector(-20, 0, 0), FVector::ZeroVector);
	Palm(F, FVector(20, 0, 0), FVector::ZeroVector, EBallHandSide::Right);
	auto S = BallTests::State();
	FBallStepOutput O;
	TestTrue(TEXT("Infeasible kinematics do not pause world"), Step(F, S, Config(), O) == EBallStepResult::Success);
	TestEqual(TEXT("Stable first collider accepted"), O.EventCount, 1);
	TestEqual(TEXT("Conflict diagnosed"), O.Diagnostics.RejectedMotionContacts, uint32(1));
	TestEqual(TEXT("No conflicting repeated impulses"), S.LinearVelocity.X, 4320.0, 1.e-6);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandInitialOverlapTest,
								 "FourSquareVR.Ball.Hand.UnknownInitialOverlapRequiresSeparation", Flags)
bool FHandInitialOverlapTest::RunTest(const FString &)
{
	FBallMotionFrame F;
	F.Epoch = BallTests::State().Identity.Epoch;
	Palm(F, FVector::ZeroVector, FVector(100, 0, 0));
	auto S = BallTests::State();
	FBallStepOutput O;
	TestTrue(TEXT("Initial overlap safe"), Step(F, S, Config(), O) == EBallStepResult::Success);
	TestEqual(TEXT("No fabricated strike"), O.EventCount, 0);
	TestTrue(TEXT("No large depenetration/hit"), S.Position.IsZero() && S.LinearVelocity.IsZero());
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandPersistentTest,
								 "FourSquareVR.Ball.Hand.PersistentContactDoesNotReapplyRestitution", Flags)
bool FHandPersistentTest::RunTest(const FString &)
{
	FBallMotionFrame F;
	F.Epoch = BallTests::State().Identity.Epoch;
	Palm(F, FVector(-10.02, 0, 0), FVector(-9.02, 0, 0));
	auto S = BallTests::State();
	S.NextEventOrdinal = 1;
	S.MotionPairCount = 1;
	S.MotionPairs[0].Collider = F.Colliders[0].Id;
	S.MotionPairs[0].Episode = 1;
	S.MotionPairs[0].EventCounterAfterImpact = 1;
	FBallStepOutput O;
	TestTrue(TEXT("Persistent solve"), Step(F, S, Config(), O) == EBallStepResult::Success);
	TestEqual(TEXT("Inelastic sustained push"), S.LinearVelocity.X, 120.0, 1.e-7);
	TestEqual(TEXT("No duplicate event"), O.EventCount, 0);
	TArray<uint8> Bytes;
	FBallState Restored;
	TestTrue(TEXT("State serializes"), FBallStateCodec::Save(S, Bytes) && FBallStateCodec::Load(Bytes, Restored));
	TestTrue(TEXT("Pair policy fully restored"), BallTests::ExactlyEqual(S, Restored));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandMotionKnotTest, "FourSquareVR.Ball.Hand.RecordedMotionKnotsPreserveReversal",
								 Flags)
bool FHandMotionKnotTest::RunTest(const FString &)
{
	FBallMotionFrame F;
	F.Epoch = BallTests::State().Identity.Epoch;
	FBallColliderMotion C;
	C.Id = BallMotionLimits::HandId(1, EBallHandSide::Left, 1);
	C.Material.bOverride = true;
	C.Material.Friction = 0;
	FBallMotionSegment M[2];
	M[0].Start = FVector(-20, 0, 0);
	M[0].End = FVector(-30, 0, 0);
	M[0].EndFraction = .5;
	M[1].Start = M[0].End;
	M[1].End = FVector::ZeroVector;
	M[1].StartFraction = .5;
	F.Add(C, MakeArrayView(M));
	auto S = BallTests::State();
	FBallStepOutput O;
	TestTrue(TEXT("Piecewise CCD"), Step(F, S, Config(), O) == EBallStepResult::Success);
	TestEqual(TEXT("Reversal TOI"), O.Events[0].Time.Fraction, 5.0 / 6, 1.e-9);
	TestEqual(TEXT("Outgoing segment velocity used"), S.LinearVelocity.X, 12960.0, 1.e-6);
	TestEqual(TEXT("Correct remainder"), S.Position.X, 18.02, 1.e-7);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandWorldSequenceTest, "FourSquareVR.Ball.Hand.PalmAndWorldImpactWithinOneStep",
								 Flags)
bool FHandWorldSequenceTest::RunTest(const FString &)
{
	FBallMotionFrame F;
	F.Epoch = BallTests::State().Identity.Epoch;
	Palm(F, FVector(-20, 0, 0), FVector::ZeroVector);
	FRecordedBallCollisionWorld Static;
	Static.AddPlane({1, 1}, FVector(20, 0, 0), FVector(-1, 0, 0));
	auto S = BallTests::State();
	FBallStepOutput O;
	TestTrue(TEXT("Combined remaining-time solve"), Step(F, S, Config(), O, &Static) == EBallStepResult::Success);
	TestEqual(TEXT("Palm then wall"), O.EventCount, 2);
	TestTrue(TEXT("Event order"),
			 O.Events[0].Category == EBallContactCategory::Hand && O.Events[1].Category != EBallContactCategory::Hand);
	TestEqual(TEXT("Final velocity"), S.LinearVelocity.X, -3456.0, 1.e-6);
	TestEqual(TEXT("Analytic final position including skins"), S.Position.X, 16.164, 1.e-6);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandSampleSafetyTest, "FourSquareVR.Ball.Hand.TrackingSafetyAndReacquisition", Flags)
bool FHandSampleSafetyTest::RunTest(const FString &)
{
	FBallHandPoseHistory H;
	H.Initialize(8);
	FBallHandTrackingSettings T;
	TestTrue(TEXT("First sample warms up"),
			 H.Add(Pose(0, FVector::ZeroVector, 1), T) == EBallTrackingRejection::WarmingUp);
	TestTrue(TEXT("Second sample continuity"),
			 H.Add(Pose(.01, FVector(1, 0, 0), 2), T) == EBallTrackingRejection::None);
	auto Lost = Pose(.02, FVector(2, 0, 0), 3);
	Lost.bTracked = false;
	TestTrue(TEXT("Tracking loss"), H.Add(Lost, T) == EBallTrackingRejection::Untracked);
	const uint32 Generation = H.GetLatest()->Generation;
	TestTrue(TEXT("Reacquire warms up"),
			 H.Add(Pose(.03, FVector(300, 0, 0), 4), T) == EBallTrackingRejection::WarmingUp);
	TestTrue(TEXT("No sweep across loss"), !H.GetLatest()->bCollisionReady);
	H.Add(Pose(.04, FVector(301, 0, 0), 5), T);
	TestEqual(TEXT("New generation retained"), H.GetLatest()->Generation, Generation);
	auto Origin = Pose(.05, FVector(1000, 0, 0), 6);
	Origin.OriginRevision = 2;
	Origin.OriginPosition.X = 700;
	TestTrue(TEXT("Teleport is a new warmup, not huge velocity"),
			 H.Add(Origin, T) == EBallTrackingRejection::WarmingUp);
	TestTrue(TEXT("Out of order ignored"),
			 H.Add(Pose(.01, FVector::ZeroVector, 2), T) == EBallTrackingRejection::OutOfOrder);
	auto Fast = Origin;
	Fast.CaptureSeconds = Fast.TimelineSeconds = .06;
	Fast.Sequence = 7;
	Fast.ControllerPosition.X += 100;
	TestTrue(TEXT("Extreme velocity rejected"), H.Add(Fast, T) == EBallTrackingRejection::ExcessiveVelocity);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandRateTest, "FourSquareVR.Ball.Hand.TimestampedRatesProduceEquivalentOutcomes",
								 Flags)
bool FHandRateTest::RunTest(const FString &)
{
	FVector Baseline;
	bool First = true;
	for (int32 Hz : {72, 90, 120, 240})
	{
		FBallHandPoseHistory P;
		P.Initialize(512);
		FBallHandTrackingSettings T;
		FBallPalmSettings PalmSettings;
		PalmSettings.Material.Friction = 0;
		for (int32 I = 0; I <= Hz * 1.2; ++I)
		{
			double Time = double(I) / Hz;
			P.Add(Pose(Time, FVector(-100 + 100 * Time, 0, 0), I + 1), T);
		}
		FBallMotionHistory H;
		H.Initialize(256);
		FRecordedBallCollisionWorld Static;
		FHistoricalBallCollisionWorld W(Static, H);
		auto S = BallTests::State();
		FBallSimulator Sim;
		FBallStepOutput O;
		int32 Events = 0;
		for (int32 F = 0; F < 132; ++F)
		{
			FBallMotionFrame M;
			M.Epoch = BallTests::State().Identity.Epoch;
			M.Frame = F;
			EBallTrackingRejection R;
			if (!P.BuildMotion(F / 120.0, PalmSettings, T, 1, EBallHandSide::Left, M, R) || !H.Push(M))
			{
				AddError(TEXT("Trajectory reconstruction failed"));
				return false;
			}
			if (Sim.SimulateStep(S, MotionInputs(S), Config(), W, O) != EBallStepResult::Success)
			{
				AddError(TEXT("Rate simulation failed"));
				return false;
			}
			Events += O.EventCount;
		}
		TestEqual(FString::Printf(TEXT("One strike at %dHz"), Hz), Events, 1);
		TestEqual(TEXT("Velocity from timestamped derivative"), S.LinearVelocity.X, 180.0, 1.e-6);
		if (First)
		{
			Baseline = S.Position;
			First = false;
		}
		else
		{
			TestTrue(TEXT("Equivalent fixed-step positions"), S.Position.Equals(Baseline, 1.e-6));
		}
	}
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandHistoricalReplayTest,
								 "FourSquareVR.Ball.Hand.HistoricalTrajectoriesReplayCompleteState", Flags)
bool FHandHistoricalReplayTest::RunTest(const FString &)
{
	FBallMotionHistory Motions;
	Motions.Initialize(64);
	FBallHistoryBuffer H;
	auto S = BallTests::State();
	H.Initialize(64, S.Identity);
	FRecordedBallCollisionWorld Static;
	Static.AddPlane({1, 1}, FVector(100, 0, 0), FVector(-1, 0, 0));
	FHistoricalBallCollisionWorld W(Static, Motions);
	FBallSimulator Sim;
	FBallStepOutput O;
	TArray<FBallCollisionEvent> OriginalEvents;
	for (int32 F = 0; F < 32; ++F)
	{
		FBallMotionFrame M;
		M.Epoch = BallTests::State().Identity.Epoch;
		M.Frame = F;
		Palm(M, FVector(F < 10 ? -1000 : F == 10 ? -20 : 0, 0, 0), FVector(F < 10 ? -1000 : 0, 0, 0));
		Motions.Push(M);
		auto I = MotionInputs(S);
		if (H.Push({S, I}) != EBallHistoryWrite::Stored ||
			Sim.SimulateStep(S, I, Config(), W, O) != EBallStepResult::Success)
		{
			AddError(TEXT("Record failed"));
			return false;
		}
		for (int32 J = 0; J < O.EventCount; ++J)
		{
			OriginalEvents.Add(O.Events[J]);
		}
	}
	FBallState Replayed = BallTests::State();
	FBallReplayRunner Runner;
	FBallReplayReport Report;
	TestTrue(TEXT("Replay actual immutable trajectories"),
			 Runner.Replay(H, S.Identity, 0, 32, Config(), {}, W, Replayed, Report) == EBallReplayResult::Success);
	TestTrue(TEXT("Every state field including episodes restored"), BallTests::ExactlyEqual(S, Replayed));
	FBallHistoryFrame First;
	H.CopyFrame(0, S.Identity, First);
	auto TraceState = First.State;
	int32 EventIndex = 0;
	for (int32 F = 0; F < 32; ++F)
	{
		FBallHistoryFrame R;
		H.CopyFrame(F, S.Identity, R);
		Sim.SimulateStep(TraceState, R.Inputs, Config(), W, O);
		for (int32 J = 0; J < O.EventCount; ++J)
		{
			if (EventIndex >= OriginalEvents.Num())
			{
				AddError(TEXT("Replay invented a duplicate event"));
				return false;
			}
			const auto &A = OriginalEvents[EventIndex++];
			const auto &B = O.Events[J];
			TestTrue(TEXT("Stable complete collision event"),
					 A.Ball == B.Ball && A.Collider == B.Collider && A.Ordinal == B.Ordinal && A.Episode == B.Episode &&
						 A.Time.Frame == B.Time.Frame && FMath::Abs(A.Time.Fraction - B.Time.Fraction) < 1.e-12 &&
						 A.Position.Equals(B.Position, 1.e-9) && A.Normal.Equals(B.Normal, 1.e-9) &&
						 A.IncomingVelocity.Equals(B.IncomingVelocity, 1.e-9) &&
						 A.OutgoingVelocity.Equals(B.OutgoingVelocity, 1.e-9) &&
						 A.IncomingAngularVelocity.Equals(B.IncomingAngularVelocity, 1.e-9) &&
						 A.OutgoingAngularVelocity.Equals(B.OutgoingAngularVelocity, 1.e-9) &&
						 A.Participant == B.Participant && A.HandSide == B.HandSide && A.FirstSample == B.FirstSample &&
						 A.SecondSample == B.SecondSample);
		}
	}
	TestEqual(TEXT("No historical events lost"), EventIndex, OriginalEvents.Num());
	const auto Before = Replayed;
	Motions.Reset();
	TestTrue(TEXT("Missing trajectories rejected before execution"),
			 Runner.Replay(H, S.Identity, 0, 32, Config(), {}, W, Replayed, Report) != EBallReplayResult::Success);
	TestEqual(TEXT("No partial replay"), Report.ExecutedFrames, uint32(0));
	TestTrue(TEXT("Destination untouched"), BallTests::ExactlyEqual(Before, Replayed));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandMotionBoundsTest, "FourSquareVR.Ball.Hand.MotionHistoryWrapPinAndOwnership",
								 Flags)
bool FHandMotionBoundsTest::RunTest(const FString &)
{
	FBallMotionHistory H;
	H.Initialize(4);
	const SIZE_T Bytes = H.GetAllocatedBytes();
	for (int32 I = 0; I < 1000; ++I)
	{
		FBallMotionFrame F;
		F.Epoch = BallTests::State().Identity.Epoch;
		F.Frame = I;
		Palm(F, FVector(I, 0, 0), FVector(I + 1, 0, 0));
		TestTrue(TEXT("Contiguous bounded write"), H.Push(F));
		F.Segments[0].Start.X = -999;
	}
	TestTrue(TEXT("Owned numeric data"), H.Find(996, 1, 1) && H.Find(996, 1, 1)->Segments[0].Start.X == 996);
	TestFalse(TEXT("Expired frame absent"), H.Find(995, 1, 1) != nullptr);
	H.PinFrom(996);
	FBallMotionFrame Next;
	Next.Epoch = BallTests::State().Identity.Epoch;
	Next.Frame = 1000;
	TestFalse(TEXT("Pinned history cannot be overwritten"), H.Push(Next));
	TestEqual(TEXT("Fixed memory"), H.GetAllocatedBytes(), Bytes);
	H.Unpin();
	TestTrue(TEXT("Unpinned write"), H.Push(Next));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandPermutation,
								 "FourSquareVR.Ball.Hand.SimultaneousInputOrderDoesNotChangeStateOrEvents", Flags)
bool FHandPermutation::RunTest(const FString &)
{
	FBallState Results[2];
	FBallStepOutput Outputs[2];
	for (int32 I = 0; I < 2; ++I)
	{
		FBallMotionFrame F;
		F.Epoch = BallTests::State().Identity.Epoch;
		if (I == 0)
		{
			Palm(F, FVector(-20, 0, 0), FVector::ZeroVector);
			Palm(F, FVector(0, -20, 0), FVector::ZeroVector, EBallHandSide::Right);
		}
		else
		{
			Palm(F, FVector(0, -20, 0), FVector::ZeroVector, EBallHandSide::Right);
			Palm(F, FVector(-20, 0, 0), FVector::ZeroVector);
		}
		Results[I] = BallTests::State();
		TestTrue(TEXT("Unordered input batch succeeds"),
				 Step(F, Results[I], Config(), Outputs[I]) == EBallStepResult::Success);
	}
	TestTrue(TEXT("Full state including pair cache is canonical"), BallTests::ExactlyEqual(Results[0], Results[1]));
	TestEqual(TEXT("Same event count"), Outputs[0].EventCount, Outputs[1].EventCount);
	for (int32 I = 0; I < Outputs[0].EventCount; ++I)
	{
		TestTrue(TEXT("Stable event ordering"), Outputs[0].Events[I].Collider == Outputs[1].Events[I].Collider &&
													Outputs[0].Events[I].Ordinal == Outputs[1].Events[I].Ordinal);
	}
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandSleepingWake, "FourSquareVR.Ball.Hand.IncomingPalmWakesSupportedRestingBall",
								 Flags)
bool FHandSleepingWake::RunTest(const FString &)
{
	FBallMotionFrame F;
	F.Epoch = BallTests::State().Identity.Epoch;
	Palm(F, FVector(-20, 0, 3.02), FVector(0, 0, 3.02));
	auto S = BallTests::State();
	S.Position.Z = 3.02;
	S.Activity = EBallActivityState::Resting;
	S.bHasSupport = true;
	S.SupportCollider = {1, 1};
	S.SleepDwellTime = 1;
	FBallPhysicsConfig C = Config();
	C.Gravity.Z = -980.665;
	FRecordedBallCollisionWorld Static;
	Static.AddPlane({1, 1}, FVector::ZeroVector, FVector::UpVector);
	FBallStepOutput O;
	TestTrue(TEXT("Sleeping broadphase includes incoming moving palm"),
			 Step(F, S, C, O, &Static) == EBallStepResult::Success);
	TestTrue(TEXT("Physical hand response wakes ball"),
			 S.Activity == EBallActivityState::Active && S.LinearVelocity.X > 4000 && S.Position.Z >= 3);
	TestEqual(TEXT("One hand event, no floor chatter"), O.EventCount, 1);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandRotatingSurface, "FourSquareVR.Ball.Hand.RotatingPalmSurfaceFrictionResponse",
								 Flags)
bool FHandRotatingSurface::RunTest(const FString &)
{
	FBallMotionFrame F;
	F.Epoch = BallTests::State().Identity.Epoch;
	Palm(F, FVector::ZeroVector, FVector::ZeroVector);
	F.Colliders[0].Material.Friction = .35;
	F.Segments[0].SurfaceAngularVelocity = FVector(0, 0, 10);
	auto S = BallTests::State();
	S.Position.X = -20;
	S.LinearVelocity.X = 2400;
	auto C = Config();
	C.SurfaceFriction = .35;
	FBallStepOutput O;
	TestTrue(TEXT("Rotating spherical surface uses shared physical response"),
			 Step(F, S, C, O) == EBallStepResult::Success);
	TestEqual(TEXT("Normal TOI"), O.Events[0].Time.Fraction, .5, 1.e-9);
	TestTrue(TEXT("Palm contact point"), O.Events[0].Position.Equals(FVector(-7, 0, 0), 1.e-8));
	TestTrue(TEXT("Normal bounce and sticking tangent response"),
			 S.LinearVelocity.Equals(FVector(-1920, -20, 0), 1.e-6));
	TestTrue(TEXT("Solid-sphere angular impulse"), S.AngularVelocity.Equals(FVector(0, 0, -50.0 / 3), 1.e-6));
	const double TangentEnergy = .5 * C.Mass * FMath::Square(S.LinearVelocity.Y) +
								 .5 * C.GetSolidSphereInertia() * S.AngularVelocity.SizeSquared();
	TestEqual(TEXT("42 units of tangential energy from 84 units motor work; remaining work dissipates"), TangentEnergy,
			  42.0, 1.e-6);
	return true;
}
#endif
