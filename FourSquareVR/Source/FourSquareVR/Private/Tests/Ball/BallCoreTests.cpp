#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "BallTestFixtures.h"
#include "Ball/BallSimulationClock.h"
#include "Ball/BallSimulator.h"
#include "Ball/RecordedBallCollisionWorld.h"
#include <limits>

namespace
{
	constexpr EAutomationTestFlags Flags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;
	bool Advance(FBallState& State, int32 Count, const FBallPhysicsConfig& Config, const IBallCollisionWorld& World)
	{
		FBallSimulator Simulator;
		FBallStepOutput Output;
		for (int32 I = 0; I < Count; ++I)
		{
			if (Simulator.SimulateStep(State, BallTests::Inputs(State), Config, World, Output) != EBallStepResult::Success) { return false; }
		}
		return true;
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallGravityTest, "FourSquareVR.Ball.Core.Gravity", Flags)
bool FBallGravityTest::RunTest(const FString& Parameters)
{
	FBallPhysicsConfig Config;
	Config.LinearDrag = Config.AngularDrag = 0;
	FRecordedBallCollisionWorld World;
	FBallState State = BallTests::State();
	TestTrue(TEXT("120 successful fixed steps"), Advance(State, 120, Config, World));
	const double Dt = Config.GetFixedDeltaTime();
	TestTrue(TEXT("Gravity velocity in cm/s"), State.LinearVelocity.Equals(Config.Gravity, 1.e-8));
	TestTrue(TEXT("Documented semi-implicit discrete position"),
		State.Position.Equals(Config.Gravity * (Dt * Dt * 120 * 121 / 2), 1.e-8));
	TestEqual(TEXT("Boundary frame"), State.SimulationFrame, uint64(120));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallRenderRatesTest, "FourSquareVR.Ball.Core.RenderingRateConsistency", Flags)
bool FBallRenderRatesTest::RunTest(const FString& Parameters)
{
	const int32 Rates[] = {30, 45, 60, 72, 90, 120, 144, 240};
	FBallPhysicsConfig Config;
	FRecordedBallCollisionWorld World;
	FBallState Reference = BallTests::State();
	Reference.AngularVelocity = FVector(0.2, 0.4, 0.8);
	TestTrue(TEXT("Reference fixed trace"), Advance(Reference, 1200, Config, World));
	for (const int32 Rate : Rates)
	{
		FBallState State = BallTests::State();
		State.AngularVelocity = FVector(0.2, 0.4, 0.8);
		FBallSimulationClock Clock;
		TestTrue(TEXT("Clock initialized"), Clock.Initialize({}, EBallClockRole::Authority));
		FBallSimulator Simulator;
		FBallStepOutput Output;
		for (int32 Frame = 0; Frame < Rate * 10; ++Frame)
		{
			TestTrue(TEXT("Valid render interval"), Clock.BeginRenderFrame(1.0 / Rate));
			while (Clock.HasStepAvailable())
			{
				if (Simulator.SimulateStep(State, BallTests::Inputs(State), Config, World, Output) != EBallStepResult::Success)
				{ AddError(TEXT("Unexpected simulator failure")); return false; }
				TestTrue(TEXT("Consume only completed step"), Clock.ConsumeCompletedStep());
			}
		}
		TestEqual(*FString::Printf(TEXT("%d Hz fixed frame count"), Rate), Clock.GetFrame(), uint64(1200));
		TestTrue(TEXT("Position within 0.001 cm"), State.Position.Equals(Reference.Position, 0.001));
		TestTrue(TEXT("Velocity within 0.01 cm/s"), State.LinearVelocity.Equals(Reference.LinearVelocity, 0.01));
		TestTrue(TEXT("Identical recorded numerical trace on this platform"), BallTests::ExactlyEqual(State, Reference));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallClockRemainderTest, "FourSquareVR.Ball.Core.ClockRemainderAndCatchUp", Flags)
bool FBallClockRemainderTest::RunTest(const FString& Parameters)
{
	FBallSimulationClock Clock;
	Clock.Initialize({}, EBallClockRole::Authority);
	const double Dt = Clock.GetFixedDeltaTime();
	Clock.BeginRenderFrame(Dt * 0.5);
	TestFalse(TEXT("Zero-step frame"), Clock.HasStepAvailable());
	Clock.BeginRenderFrame(Dt * 0.5);
	TestTrue(TEXT("Remainder combines"), Clock.ConsumeCompletedStep());
	Clock.BeginRenderFrame(Dt * 12.5);
	int32 Count = 0;
	while (Clock.ConsumeCompletedStep()) { ++Count; }
	TestEqual(TEXT("Catch-up bounded"), Count, 8);
	TestTrue(TEXT("All unconsumed debt retained"), FMath::IsNearlyEqual(Clock.GetAccumulatorSeconds(), 4.5 * Dt, 1.e-12));
	Clock.BeginRenderFrame(0);
	Count = 0;
	while (Clock.ConsumeCompletedStep()) { ++Count; }
	TestEqual(TEXT("Debt recovered next render frame without larger dt"), Count, 4);
	TestTrue(TEXT("Fraction retained"), FMath::IsNearlyEqual(Clock.GetAccumulatorSeconds(), 0.5 * Dt, 1.e-12));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallClockOverrunTest, "FourSquareVR.Ball.Core.ClockOverrunRecovery", Flags)
bool FBallClockOverrunTest::RunTest(const FString& Parameters)
{
	FBallSimulationClock Authority;
	Authority.Initialize({}, EBallClockRole::Authority, 50);
	TestFalse(TEXT("Large stall pauses authority"), Authority.BeginRenderFrame(0.5));
	TestTrue(TEXT("Explicit paused status"), Authority.GetStatus() == EBallClockStatus::AuthorityPaused);
	TestEqual(TEXT("No fabricated steps"), Authority.GetFrame(), uint64(50));
	TestEqual(TEXT("Debt retained pending explicit decision"), Authority.GetAccumulatorSeconds(), 0.5);
	TestFalse(TEXT("Cannot step paused"), Authority.ConsumeCompletedStep());
	TestFalse(TEXT("Same timebase cannot erase debt"), Authority.ResumeAuthority(1));
	TestTrue(TEXT("Explicit new timebase resumes"), Authority.ResumeAuthority(2));
	TestEqual(TEXT("Discarded pause accounted"), Authority.GetDiagnostics().ExplicitlyDiscardedSeconds, 0.5);
	TestEqual(TEXT("Resume does not advance simulation"), Authority.GetFrame(), uint64(50));
	FBallSimulationClock Client;
	Client.Initialize({}, EBallClockRole::PredictedClient);
	Client.BeginRenderFrame(1);
	TestTrue(TEXT("Client requests baseline"), Client.GetStatus() == EBallClockStatus::NeedsBaseline);
	TestFalse(TEXT("Client cannot resume as authority"), Client.ResumeAuthority(2));
	TestTrue(TEXT("Actual baseline recovers client"), Client.ApplyClientBaseline(90, 2));
	TestEqual(TEXT("Baseline frame"), Client.GetFrame(), uint64(90));
	TestFalse(TEXT("Old timebase rejected"), Client.ApplyClientBaseline(91, 1));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallClockInvalidTest, "FourSquareVR.Ball.Core.ClockInvalidInput", Flags)
bool FBallClockInvalidTest::RunTest(const FString& Parameters)
{
	FBallSimulationClock Clock;
	TestFalse(TEXT("Uninitialized clock"), Clock.BeginRenderFrame(0.01));
	FBallClockSettings Settings;
	Settings.FixedFrequency = 0;
	TestFalse(TEXT("Zero Hz invalid"), Clock.Initialize(Settings, EBallClockRole::Authority));
	Clock.Initialize({}, EBallClockRole::Authority);
	TestFalse(TEXT("Negative time rejected"), Clock.BeginRenderFrame(-1));
	TestFalse(TEXT("NaN time rejected"), Clock.BeginRenderFrame(std::numeric_limits<double>::quiet_NaN()));
	TestFalse(TEXT("Infinite time rejected"), Clock.BeginRenderFrame(std::numeric_limits<double>::infinity()));
	TestEqual(TEXT("Invalid input did not alter debt"), Clock.GetAccumulatorSeconds(), 0.0);
	TestEqual(TEXT("Invalid input did not advance frame"), Clock.GetFrame(), uint64(0));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallConfigAndFinalityTest, "FourSquareVR.Ball.Core.ConfigurationAndFinality", Flags)
bool FBallConfigAndFinalityTest::RunTest(const FString& Parameters)
{
	FBallPhysicsConfig Physics;
	TestTrue(TEXT("Defaults valid"), Physics.IsValid());
	TestTrue(TEXT("Sphere inertia units"), FMath::IsNearlyEqual(Physics.GetSolidSphereInertia(), 0.216, 1.e-12));
	Physics.Mass = 0;
	TestFalse(TEXT("Zero mass rejected"), Physics.IsValid());
	Physics = {};
	Physics.Restitution = 1.1;
	TestFalse(TEXT("Passive restitution bounded"), Physics.IsValid());
	FBallNetworkSettings Settings;
	TestTrue(TEXT("Default horizon fits capacities"), Settings.IsValid(120));
	TestEqual(TEXT("250 ms is thirty ticks"), Settings.GetRevisionFrames(120), 30);
	uint64 Boundary = 999;
	TestTrue(TEXT("Eligible frontier computed"), FBallFinalityPolicy::CalculateEligibleBoundary(100, Settings, 120, Boundary));
	TestEqual(TEXT("Only frame indices below 70 eligible"), Boundary, uint64(70));
	Settings.RevisionWindowSeconds = 0.251;
	TestEqual(TEXT("Fractional frame rounds conservatively"), Settings.GetRevisionFrames(120), 31);
	Settings.RevisionWindowSeconds = 0.5;
	TestFalse(TEXT("Increasing horizon requires replay budget"), Settings.IsValid(120));
	Settings.MaximumReplayFrames = 64;
	TestTrue(TEXT("Larger window supported by sufficient capacity"), Settings.IsValid(120));
	FBallFinalityPolicy::CalculateEligibleBoundary(100, Settings, 120, Boundary);
	TestEqual(TEXT("Configurable finality changes eligibility"), Boundary, uint64(40));
	FBallFinalityPolicy::CalculateEligibleBoundary(10, Settings, 120, Boundary);
	TestEqual(TEXT("No unsigned underflow at startup"), Boundary, uint64(0));
	Settings.RevisionWindowSeconds = std::numeric_limits<double>::infinity();
	Boundary = 999;
	TestFalse(TEXT("Invalid policy rejected"), FBallFinalityPolicy::CalculateEligibleBoundary(100, Settings, 120, Boundary));
	TestEqual(TEXT("Failure leaves destination"), Boundary, uint64(999));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallSequenceTest, "FourSquareVR.Ball.Core.SequenceWrap", Flags)
bool FBallSequenceTest::RunTest(const FString& Parameters)
{
	TestTrue(TEXT("Normal newer"), FBallFinalityPolicy::IsSequenceNewer(12, 11));
	TestFalse(TEXT("Duplicate"), FBallFinalityPolicy::IsSequenceNewer(11, 11));
	TestTrue(TEXT("Wrap newer"), FBallFinalityPolicy::IsSequenceNewer(1, MAX_uint32));
	TestFalse(TEXT("Old wrapped packet"), FBallFinalityPolicy::IsSequenceNewer(MAX_uint32, 1));
	TestFalse(TEXT("Half-range ambiguous"), FBallFinalityPolicy::IsSequenceNewer(0x80000000, 0));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallDampingAndRotationTest, "FourSquareVR.Ball.Core.DampingAndAngularIntegration", Flags)
bool FBallDampingAndRotationTest::RunTest(const FString& Parameters)
{
	FBallPhysicsConfig Config;
	Config.Gravity = FVector::ZeroVector;
	Config.LinearDrag = 1;
	Config.AngularDrag = 0;
	FBallState State = BallTests::State();
	State.LinearVelocity = FVector(100, 0, 0);
	State.AngularVelocity = FVector(0, 0, UE_DOUBLE_PI);
	FRecordedBallCollisionWorld World;
	TestTrue(TEXT("Damped trace succeeds"), Advance(State, 120, Config, World));
	TestTrue(TEXT("Exponential drag analytic result"), FMath::IsNearlyEqual(State.LinearVelocity.X, 100 * FMath::Exp(-1.0), 1.e-9));
	TestTrue(TEXT("Drag never reverses"), State.LinearVelocity.X > 0);
	TestTrue(TEXT("World angular integration"), State.Rotation.AngularDistance(FQuat(FVector::UpVector, UE_DOUBLE_PI)) < 1.e-6);
	Config.LinearDrag = 0;
	Config.MaximumLinearSpeed = 10;
	Config.MaximumAngularSpeed = 0.5;
	State = BallTests::State();
	State.LinearVelocity = FVector(100, 0, 0);
	State.AngularVelocity = FVector(0, 0, 10);
	TestTrue(TEXT("Designer speed caps"), Advance(State, 1, Config, World));
	TestTrue(TEXT("Linear cap"), FMath::IsNearlyEqual(State.LinearVelocity.Size(), 10.0, 1.e-8));
	TestTrue(TEXT("Angular cap"), FMath::IsNearlyEqual(State.AngularVelocity.Size(), 0.5, 1.e-8));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallZeroAndFiniteTest, "FourSquareVR.Ball.Core.ZeroAndInvalidState", Flags)
bool FBallZeroAndFiniteTest::RunTest(const FString& Parameters)
{
	FBallPhysicsConfig Config;
	Config.Gravity = FVector::ZeroVector;
	FRecordedBallCollisionWorld World;
	FBallState State = BallTests::State();
	TestTrue(TEXT("Zero velocity trace"), Advance(State, 120, Config, World));
	TestTrue(TEXT("Zero remains zero"), State.Position.IsZero() && State.LinearVelocity.IsZero());
	FBallSimulator Simulator;
	FBallStepOutput Output;
	State.Position.X = std::numeric_limits<double>::quiet_NaN();
	TestTrue(TEXT("NaN rejected"), Simulator.SimulateStep(State, BallTests::Inputs(State), Config, World, Output) == EBallStepResult::InvalidState);
	TestEqual(TEXT("Failure did not advance frame"), State.SimulationFrame, uint64(120));
	State = BallTests::State();
	State.LinearVelocity.X = std::numeric_limits<double>::max();
	const FBallState Before = State;
	TestTrue(TEXT("Overflow rejected"), Simulator.SimulateStep(State, BallTests::Inputs(State), Config, World, Output) == EBallStepResult::NumericFailure);
	TestTrue(TEXT("Overflow transaction unchanged"), BallTests::ExactlyEqual(State, Before));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallInputsTest, "FourSquareVR.Ball.Core.RecordedForceOrderingAndVersions", Flags)
bool FBallInputsTest::RunTest(const FString& Parameters)
{
	FBallPhysicsConfig Config;
	Config.Gravity = FVector::ZeroVector;
	Config.LinearDrag = 0;
	FBallState State = BallTests::State();
	FBallStepInputs Inputs = BallTests::Inputs(State);
	Inputs.ForceCount = 2;
	Inputs.Forces[0] = {1, 0, FVector(120, 0, 0), FVector::ZeroVector};
	Inputs.Forces[1] = {2, 1, FVector(0, 240, 0), FVector::ZeroVector};
	FBallSimulator Simulator;
	FBallStepOutput Output;
	FRecordedBallCollisionWorld World;
	TestTrue(TEXT("Ordered recorded forces"), Simulator.SimulateStep(State, Inputs, Config, World, Output) == EBallStepResult::Success);
	TestTrue(TEXT("Forces integrated once"), State.LinearVelocity.Equals(FVector(1, 2, 0), 1.e-9));
	Inputs = BallTests::Inputs(State);
	Inputs.ForceCount = 2;
	Inputs.Forces[0].ProviderId = Inputs.Forces[1].ProviderId = 1;
	const FBallState Before = State;
	TestTrue(TEXT("Duplicate provider input rejected"), Simulator.SimulateStep(State, Inputs, Config, World, Output) == EBallStepResult::InvalidInputs);
	Inputs = BallTests::Inputs(State);
	++Inputs.SimulationFrame;
	TestTrue(TEXT("Wrong frame rejected"), Simulator.SimulateStep(State, Inputs, Config, World, Output) == EBallStepResult::InvalidInputs);
	Inputs = BallTests::Inputs(State);
	Inputs.EnvironmentRevision = 2;
	TestTrue(TEXT("World revision mismatch rejected"), Simulator.SimulateStep(State, Inputs, Config, World, Output) == EBallStepResult::IncompatibleEnvironment);
	TestTrue(TEXT("All rejection paths preserve state"), BallTests::ExactlyEqual(Before, State));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallSubdivisionTest, "FourSquareVR.Ball.Core.SubdivisionAndFailureAtomicity", Flags)
bool FBallSubdivisionTest::RunTest(const FString& Parameters)
{
	FBallPhysicsConfig Config;
	Config.Gravity = FVector(0, 0, -10000);
	Config.MaximumMicrointervals = 1;
	FBallState State = BallTests::State();
	const FBallState Before = State;
	FBallSimulator Simulator;
	FBallStepOutput Output;
	FRecordedBallCollisionWorld World;
	TestTrue(TEXT("Cannot silently violate error budget"), Simulator.SimulateStep(State, BallTests::Inputs(State), Config, World, Output) == EBallStepResult::SubdivisionLimit);
	TestTrue(TEXT("No partial state"), BallTests::ExactlyEqual(State, Before));
	Config.MaximumMicrointervals = 8;
	TestTrue(TEXT("Bounded finer intervals succeed"), Simulator.SimulateStep(State, BallTests::Inputs(State), Config, World, Output) == EBallStepResult::Success);
	TestTrue(TEXT("More than one microinterval"), Output.Diagnostics.Microintervals > 1 && Output.Diagnostics.Microintervals <= 8);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallAnalyticQueryTest, "FourSquareVR.Ball.Core.AnalyticQueryAndUnsupportedContact", Flags)
bool FBallAnalyticQueryTest::RunTest(const FString& Parameters)
{
	FRecordedBallCollisionWorld World;
	TestTrue(TEXT("Fixture plane added"), World.AddPlane({9, 1}, FVector::ZeroVector, FVector::UpVector));
	TestTrue(TEXT("Tie plane added in reverse ID order"), World.AddPlane({2, 1}, FVector::ZeroVector, FVector::UpVector));
	FBallCollisionQuery Query;
	Query.Start = FVector(0, 0, 10);
	Query.End = FVector(0, 0, -10);
	Query.Radius = 3;
	FBallContact Contact;
	TestTrue(TEXT("Analytic swept contact"), World.SweepSphere(Query, Contact) == EBallQueryResult::Contact);
	TestTrue(TEXT("Correct TOI"), FMath::IsNearlyEqual(Contact.TimeOfImpact, 0.35, 1.e-12));
	TestEqual(TEXT("Stable tie order"), Contact.Collider.Value, uint64(2));
	TestTrue(TEXT("Contact on plane"), Contact.Position.Equals(FVector::ZeroVector, 1.e-10));
	TestFalse(TEXT("Queried fixture cannot mutate under same revision"), World.AddPlane({3, 1}, FVector(0, 0, -5), FVector::UpVector));
	FBallPhysicsConfig Config;
	Config.Gravity = FVector::ZeroVector;
	Config.LinearDrag = 0;
	FBallState State = BallTests::State();
	State.Position.Z = 10;
	State.LinearVelocity.Z = -2400;
	const FBallState Before = State;
	FBallSimulator Simulator;
	FBallStepOutput Output;
	TestTrue(TEXT("Stage 3 response explicitly required"), Simulator.SimulateStep(State, BallTests::Inputs(State), Config, World, Output) == EBallStepResult::ContactSolverRequired);
	TestTrue(TEXT("No silent tunnelling or partial movement"), BallTests::ExactlyEqual(State, Before));
	TestEqual(TEXT("Contact returned for diagnostics"), Output.ContactCount, 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallInstrumentationTest, "FourSquareVR.Ball.Core.PerformanceCounters", Flags)
bool FBallInstrumentationTest::RunTest(const FString& Parameters)
{
	FBallSimulator Simulator;
	FRecordedBallCollisionWorld World;
	FBallPhysicsConfig Config;
	FBallState State = BallTests::State();
	FBallStepOutput Output;
	Simulator.SimulateStep(State, BallTests::Inputs(State), Config, World, Output);
	Simulator.SimulateStep(State, BallTests::Inputs(State), Config, World, Output);
	FBallStepInputs Bad = BallTests::Inputs(State);
	++Bad.SimulationFrame;
	Simulator.SimulateStep(State, Bad, Config, World, Output);
	const FBallPerformanceCounters& Counters = Simulator.GetPerformanceCounters();
	TestEqual(TEXT("Every attempted step counted"), Counters.Steps, uint64(3));
	TestEqual(TEXT("Failures separate"), Counters.FailedSteps, uint64(1));
	TestEqual(TEXT("Queries counted"), Counters.Queries, uint64(2));
	TestTrue(TEXT("Measured elapsed time finite"), FMath::IsFinite(Counters.TotalStepMicroseconds) && Counters.TotalStepMicroseconds >= 0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallConfigurableFrequencyTest, "FourSquareVR.Ball.Core.ConfigurableFixedFrequency", Flags)
bool FBallConfigurableFrequencyTest::RunTest(const FString& Parameters)
{
	for (const double Frequency : {60.0, 100.0, 240.0})
	{
		FBallPhysicsConfig Config;
		Config.FixedFrequency = Frequency;
		Config.Gravity = FVector(0, 0, -10);
		Config.LinearDrag = Config.AngularDrag = 0;
		FBallClockSettings Settings;
		Settings.FixedFrequency = Frequency;
		FBallSimulationClock Clock;
		Clock.Initialize(Settings, EBallClockRole::Authority);
		FBallState State = BallTests::State();
		FBallSimulator Simulator;
		FBallStepOutput Output;
		FRecordedBallCollisionWorld World;
		for (int32 Frame = 0; Frame < 90; ++Frame)
		{
			Clock.BeginRenderFrame(1.0 / 90);
			while (Clock.HasStepAvailable())
			{
				if (Simulator.SimulateStep(State, BallTests::Inputs(State), Config, World, Output) != EBallStepResult::Success)
				{ AddError(TEXT("Configured fixed-frequency step failed")); return false; }
				Clock.ConsumeCompletedStep();
			}
		}
		TestEqual(TEXT("Frequency changes actual fixed-step count"), Clock.GetFrame(), static_cast<uint64>(Frequency));
		TestTrue(TEXT("Configured dt used in integrator"), State.LinearVelocity.Equals(Config.Gravity, 1.e-8));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallPersistentStateAndQueryFailureTest, "FourSquareVR.Ball.Core.PersistentStateAndQueryFailure", Flags)
bool FBallPersistentStateAndQueryFailureTest::RunTest(const FString& Parameters)
{
	FBallState State = BallTests::RichState();
	const FBallState Before = State;
	FBallPhysicsConfig Config;
	Config.Version = State.ConfigVersion;
	FRecordedBallCollisionWorld World(State.EnvironmentRevision);
	FBallSimulator Simulator;
	FBallStepOutput Output;
	TestTrue(TEXT("Persistent state never silently discarded by free-flight foundation"),
		Simulator.SimulateStep(State, BallTests::Inputs(State), Config, World, Output) == EBallStepResult::ContactSolverRequired);
	TestTrue(TEXT("All cached support/contact/provider data preserved"), BallTests::ExactlyEqual(State, Before));
	State.bHasSupport = false;
	TestFalse(TEXT("Resting without support is invalid"), State.IsValid());
	class FQueryFault final : public IBallCollisionWorld
	{
	public:
		virtual uint32 GetRevision() const override { return 1; }
		virtual EBallQueryResult SweepSphere(const FBallCollisionQuery&, FBallContact&) const override { return EBallQueryResult::InvalidQuery; }
	} FaultWorld;
	State = BallTests::State();
	const FBallState ActiveBefore = State;
	Config = {};
	TestTrue(TEXT("Query failure propagated"), Simulator.SimulateStep(State, BallTests::Inputs(State), Config, FaultWorld, Output) == EBallStepResult::QueryFailure);
	TestTrue(TEXT("Query failure does not apply kick or drift"), BallTests::ExactlyEqual(State, ActiveBefore));
	return true;
}

#endif
