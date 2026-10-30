#include "Misc/AutomationTest.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "BallTestFixtures.h"
#include "Ball/BallCollisionSolver.h"
#include "Ball/BallSimulator.h"
#include "Ball/RecordedBallCollisionWorld.h"
#include "Ball/BallSimulationClock.h"
#include "Ball/BallReplayRunner.h"
#include "HAL/PlatformTime.h"

namespace
{
constexpr auto CollisionFlags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;
FBallPhysicsConfig Passive(double Restitution = 1, double Friction = 0)
{
	FBallPhysicsConfig C;
	C.Gravity = FVector::ZeroVector;
	C.LinearDrag = C.AngularDrag = 0;
	C.Restitution = Restitution;
	C.SurfaceFriction = Friction;
	C.SpinFriction = C.RollingResistance = 0;
	return C;
}
bool Step(FBallState& State, const FBallPhysicsConfig& C, const IBallCollisionWorld& W, FBallStepOutput& Out)
{
	FBallSimulator Simulator;
	return Simulator.SimulateStep(State, BallTests::Inputs(State), C, W, Out) == EBallStepResult::Success;
}
double Kinetic(const FBallState& S, const FBallPhysicsConfig& C)
{
	return 0.5 *
		   (C.Mass * S.LinearVelocity.SizeSquared() + C.GetSolidSphereInertia() * S.AngularVelocity.SizeSquared());
}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallHighSpeedPlanes, "FourSquareVR.Ball.Collision.HighSpeedFloorWallCeiling",
								 CollisionFlags)
bool FBallHighSpeedPlanes::RunTest(const FString&)
{
	for (const FVector N : {FVector::UpVector, FVector::ForwardVector, -FVector::UpVector})
	{
		FRecordedBallCollisionWorld W;
		W.AddPlane({1, 1}, FVector::ZeroVector, N);
		auto C = Passive(0.8);
		auto S = BallTests::State();
		S.Position = N * 100;
		S.LinearVelocity = -N * 24000;
		FBallStepOutput O;
		TestTrue(TEXT("High-speed step succeeds"), Step(S, C, W, O));
		TestTrue(TEXT("Normal velocity restitution"), S.LinearVelocity.Equals(N * 19200, 1.e-6));
		TestTrue(TEXT("Analytic remainder position"),
				 S.Position.Equals(N * (3 + C.PenetrationTolerance + 0.8 * 103), 1.e-6));
		TestEqual(TEXT("One impact"), O.EventCount, 1);
		TestTrue(TEXT("TOI 97/200"), FMath::IsNearlyEqual(O.Events[0].Time.Fraction, 97.0 / 200, 1.e-10));
		TestTrue(TEXT("Normal and point"),
				 O.Events[0].Normal.Equals(N, 1.e-10) && O.Events[0].Position.IsNearlyZero(1.e-8));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallThinWall, "FourSquareVR.Ball.Collision.ThinWallNoTunnelling", CollisionFlags)
bool FBallThinWall::RunTest(const FString&)
{
	for (double Speed : {24000.0, 120000.0, 600000.0})
	{
		FRecordedBallCollisionWorld W;
		W.AddBox({1, 1}, FVector(0, -100, -100), FVector(.1, 100, 100));
		auto C = Passive();
		auto S = BallTests::State();
		S.Position = FVector(-50, 0, 0);
		S.LinearVelocity = FVector(Speed, 0, 0);
		FBallStepOutput O;
		TestTrue(TEXT("Thin box swept at extreme speed"), Step(S, C, W, O));
		TestTrue(TEXT("Reflected, never on far side"),
				 S.Position.X < -3 && FMath::IsNearlyEqual(S.LinearVelocity.X, -Speed, 1.e-5));
		TestTrue(TEXT("Exact position with remaining time"),
				 FMath::IsNearlyEqual(S.Position.X, 44 - Speed / 120 - C.PenetrationTolerance, 1.e-5));
		TestTrue(TEXT("Analytic TOI"), FMath::IsNearlyEqual(O.Events[0].Time.Fraction, 47 * 120 / Speed, 1.e-9));
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallGlancing, "FourSquareVR.Ball.Collision.GlancingAndNearTangent", CollisionFlags)
bool FBallGlancing::RunTest(const FString&)
{
	FRecordedBallCollisionWorld W;
	W.AddPlane({1, 1}, FVector::ZeroVector, FVector::UpVector);
	auto C = Passive(.5);
	auto S = BallTests::State();
	S.Position = FVector(0, 0, 3.1);
	S.LinearVelocity = FVector(30000, 0, -30);
	FBallStepOutput O;
	TestTrue(TEXT("Glancing hit"), Step(S, C, W, O));
	TestTrue(TEXT("Tangential velocity conserved"), S.LinearVelocity.Equals(FVector(30000, 0, 15), 1.e-7));
	TestTrue(TEXT("Glancing position analytic"),
			 S.Position.Equals(FVector(250, 0, 3 + C.PenetrationTolerance + .075), 1.e-7));
	for (double Height : {3.0, 3.0000001})
	{
		S = BallTests::State();
		S.Position = FVector(0, 0, Height);
		S.LinearVelocity = FVector(30000, 0, 0);
		TestTrue(TEXT("Tangential contact does not loop"), Step(S, C, W, O));
		TestTrue(TEXT("Tangent straight path"), S.Position.Equals(FVector(250, 0, Height), 1.e-8));
		TestEqual(TEXT("No tangent impulse"), O.EventCount, 0);
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallSphereKernel, "FourSquareVR.Ball.Collision.RelativeSphereTOIKernel",
								 CollisionFlags)
bool FBallSphereKernel::RunTest(const FString&)
{
	double T = 99;
	TestTrue(TEXT("Two moving spheres in relative coordinates"),
			 FBallCollisionSolver::SphereTOI(FVector(-10, 0, 0), FVector(20, 0, 0), 4, T));
	TestTrue(TEXT("Combined radius, TOI 0.3"), FMath::IsNearlyEqual(T, .3, 1.e-12));
	TestFalse(TEXT("Exact tangency needs no impulse"),
			  FBallCollisionSolver::SphereTOI(FVector(-10, 4, 0), FVector(20, 0, 0), 4, T));
	TestFalse(TEXT("Stationary relative motion"),
			  FBallCollisionSolver::SphereTOI(FVector(-10, 0, 0), FVector::ZeroVector, 4, T));
	TestFalse(TEXT("Separating path"), FBallCollisionSolver::SphereTOI(FVector(-10, 0, 0), FVector(-20, 0, 0), 4, T));
	FBallCollisionQuery Q;
	Q.Start = FVector(-10, 3.999, 0);
	Q.End = FVector(10, 3.999, 0);
	Q.Radius = 2;
	FBallContact Hit;
	Hit.Collider = {1, 1};
	TestTrue(TEXT("Near-tangent analytic sphere"), FBallCollisionSolver::SweepSphere(Q, FVector::ZeroVector, 2, Hit));
	const double X = -FMath::Sqrt(16 - 3.999 * 3.999);
	TestTrue(TEXT("Near-tangent root accuracy"), FMath::IsNearlyEqual(Hit.TimeOfImpact, (X + 10) / 20, 1.e-10));
	TestTrue(TEXT("Normal matches sphere radius"), Hit.Normal.Equals(FVector(X, 3.999, 0) / 4, 1.e-9));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallMultiImpact, "FourSquareVR.Ball.Collision.FloorThenWallRemainingTime",
								 CollisionFlags)
bool FBallMultiImpact::RunTest(const FString&)
{
	FRecordedBallCollisionWorld W;
	W.AddPlane({2, 1}, FVector(20, 0, 0), -FVector::ForwardVector);
	W.AddPlane({1, 1}, FVector::ZeroVector, FVector::UpVector);
	auto C = Passive();
	auto S = BallTests::State();
	S.Position = FVector(0, 0, 10);
	S.LinearVelocity = FVector(3000, 0, -3000);
	FBallStepOutput O;
	TestTrue(TEXT("Floor then wall in 8.333ms"), Step(S, C, W, O));
	TestEqual(TEXT("Two ordered strikes"), O.EventCount, 2);
	TestTrue(TEXT("Final reflected velocity"), S.LinearVelocity.Equals(FVector(-3000, 0, 3000), 1.e-7));
	TestTrue(TEXT("Both remainders accounted"),
			 S.Position.Equals(FVector(9 - C.PenetrationTolerance, 0, 21 + C.PenetrationTolerance), 1.e-7));
	TestTrue(TEXT("Independent event times"), FMath::IsNearlyEqual(O.Events[0].Time.Fraction, 7.0 / 25, 1.e-10) &&
												  FMath::IsNearlyEqual(O.Events[1].Time.Fraction, 17.0 / 25, 1.e-10));
	TestEqual(TEXT("Monotonic event ordinals"), O.Events[1].Ordinal, O.Events[0].Ordinal + 1);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallSimultaneousCorner, "FourSquareVR.Ball.Collision.SimultaneousCornerOrder",
								 CollisionFlags)
bool FBallSimultaneousCorner::RunTest(const FString&)
{
	FBallState Reference;
	for (int32 Order = 0; Order < 2; ++Order)
	{
		FRecordedBallCollisionWorld W;
		if (Order)
		{
			W.AddPlane({2, 1}, FVector::ZeroVector, FVector::ForwardVector);
		}
		W.AddPlane({1, 1}, FVector::ZeroVector, FVector::UpVector);
		if (!Order)
		{
			W.AddPlane({2, 1}, FVector::ZeroVector, FVector::ForwardVector);
		}
		auto C = Passive();
		auto S = BallTests::State();
		S.Position = FVector(10, 0, 10);
		S.LinearVelocity = FVector(-2400, 0, -2400);
		FBallStepOutput O;
		TestTrue(TEXT("Simultaneous manifold succeeds"), Step(S, C, W, O));
		TestTrue(TEXT("Both axes reflect"), S.LinearVelocity.Equals(FVector(2400, 0, 2400), 1.e-7));
		TestTrue(TEXT("Corner position"),
				 S.Position.Equals(FVector(16 + C.PenetrationTolerance, 0, 16 + C.PenetrationTolerance), 1.e-7));
		TestEqual(TEXT("Two events"), O.EventCount, 2);
		TestTrue(TEXT("Strict ID order"), O.Events[0].Collider.Value < O.Events[1].Collider.Value);
		if (Order)
		{
			TestTrue(TEXT("Insertion order independent full state"), BallTests::ExactlyEqual(S, Reference));
		}
		else
		{
			Reference = S;
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallRoundedBoxFeatures, "FourSquareVR.Ball.Collision.RoundedBoxEdgeAndVertex",
								 CollisionFlags)
bool FBallRoundedBoxFeatures::RunTest(const FString&)
{
	const FVector Min(0, 0, -1), Max(1, 1, 1);
	FBallCollisionQuery Q;
	Q.Start = FVector(-5, -5, 0);
	Q.End = FVector(5, 5, 0);
	Q.Radius = 1;
	FBallContact Hit;
	TestTrue(TEXT("Finite edge cylinder"), FBallCollisionSolver::SweepBox(Q, Min, Max, Hit));
	TestTrue(TEXT("Rounded edge TOI, not expanded AABB"),
			 FMath::IsNearlyEqual(Hit.TimeOfImpact, (5 - 1 / FMath::Sqrt(2.0)) / 10, 1.e-12));
	TestTrue(TEXT("Edge point and diagonal normal"),
			 Hit.Position.IsNearlyZero(1.e-9) && Hit.Normal.Equals(FVector(-1, -1, 0).GetSafeNormal(), 1.e-10));
	Q.Start = FVector(-5, -5, -5);
	Q.End = FVector(5, 5, 5);
	TestTrue(TEXT("Vertex sphere"), FBallCollisionSolver::SweepBox(Q, FVector::ZeroVector, FVector(1), Hit));
	TestTrue(TEXT("Vertex TOI"), FMath::IsNearlyEqual(Hit.TimeOfImpact, (5 - 1 / FMath::Sqrt(3.0)) / 10, 1.e-12));
	TestTrue(TEXT("Vertex point/normal"),
			 Hit.Position.IsNearlyZero(1.e-9) && Hit.Normal.Equals(FVector(-1, -1, -1).GetSafeNormal(), 1.e-10));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallPenetration, "FourSquareVR.Ball.Collision.InitialOverlapAndZeroTime",
								 CollisionFlags)
bool FBallPenetration::RunTest(const FString&)
{
	FRecordedBallCollisionWorld W;
	W.AddPlane({1, 1}, FVector::ZeroVector, FVector::UpVector);
	auto C = Passive();
	auto S = BallTests::State();
	S.Position.Z = 1;
	S.LinearVelocity.Z = -100;
	FBallStepOutput O;
	TestTrue(TEXT("Initial overlap resolves"), Step(S, C, W, O));
	TestTrue(TEXT("Depenetration position"), FMath::IsNearlyEqual(S.Position.Z, 3 + C.PenetrationTolerance, 1.e-9));
	TestTrue(TEXT("Deep overlap cannot create bounce energy"), S.LinearVelocity.IsNearlyZero(1.e-9));
	TestEqual(TEXT("No fabricated strike"), O.EventCount, 0);
	S = BallTests::State();
	S.Position.Z = 3;
	S.LinearVelocity.Z = -1200;
	TestTrue(TEXT("Zero-time incoming contact"), Step(S, C, W, O));
	TestTrue(TEXT("Full remaining time used at t=0"),
			 FMath::IsNearlyEqual(S.Position.Z, 13 + C.PenetrationTolerance, 1.e-8));
	TestEqual(TEXT("Single zero-time event"), O.EventCount, 1);
	TestEqual(TEXT("Exact zero-time"), O.Events[0].Time.Fraction, 0.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallResting, "FourSquareVR.Ball.Collision.ExtendedRestAndWake", CollisionFlags)
bool FBallResting::RunTest(const FString&)
{
	FRecordedBallCollisionWorld W;
	W.AddPlane({1, 1}, FVector::ZeroVector, FVector::UpVector);
	FBallPhysicsConfig C;
	auto S = BallTests::State();
	S.Position.Z = 3.02;
	FBallSimulator Simulator;
	FBallStepOutput O;
	double MaximumSpeed = 0;
	for (int32 I = 0; I < 7200; ++I)
	{
		if (Simulator.SimulateStep(S, BallTests::Inputs(S), C, W, O) != EBallStepResult::Success)
		{
			AddError(TEXT("Resting trace failed"));
			return false;
		}
		MaximumSpeed = FMath::Max(MaximumSpeed, S.LinearVelocity.Size());
		if (S.Position.Z < 3 - 1.e-8 || S.Position.Z > 3.041)
		{
			AddError(TEXT("Resting ball left bounded support slop"));
			return false;
		}
	}
	TestTrue(TEXT("Sleeps after dwell"), S.Activity == EBallActivityState::Resting && S.bHasSupport);
	TestTrue(TEXT("Exact zero sleeping velocities"), S.LinearVelocity.IsZero() && S.AngularVelocity.IsZero());
	TestTrue(TEXT("No restitution chatter energy"), MaximumSpeed < 1.e-6);
	auto Inputs = BallTests::Inputs(S);
	Inputs.ForceCount = 1;
	Inputs.Forces[0] = {1, 0, FVector(0, 0, 2000), FVector::ZeroVector};
	TestTrue(TEXT("Recorded upward force wakes"),
			 Simulator.SimulateStep(S, Inputs, C, W, O) == EBallStepResult::Success);
	TestTrue(TEXT("Awake and moving upward"), S.Activity == EBallActivityState::Active && S.LinearVelocity.Z > 8);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallSlope, "FourSquareVR.Ball.Collision.SlopeSupportAndRolling", CollisionFlags)
bool FBallSlope::RunTest(const FString&)
{
	const FVector N = FVector(.6, 0, .8);
	FRecordedBallCollisionWorld W;
	W.AddPlane({1, 1}, FVector::ZeroVector, N);
	auto C = Passive();
	C.Gravity = FVector(0, 0, -980.665);
	auto S = BallTests::State();
	S.Position = N * 3.02;
	FBallStepOutput O;
	for (int32 I = 0; I < 120; ++I)
	{
		if (!Step(S, C, W, O))
		{
			AddError(TEXT("Slope trace failed"));
			return false;
		}
	}
	TestTrue(TEXT("Frictionless slope does not sleep"),
			 S.Activity == EBallActivityState::Active && S.LinearVelocity.Size() > 500);
	TestTrue(TEXT("No slope tunnelling"), FVector::DotProduct(S.Position, N) >= 3 - 1.e-6);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallEnergy, "FourSquareVR.Ball.Collision.RestitutionFrictionEnergy", CollisionFlags)
bool FBallEnergy::RunTest(const FString&)
{
	for (double E : {0.0, .5, 1.0})
		for (double Mu : {0.0, .3, 1.0})
		{
			FRecordedBallCollisionWorld W;
			W.AddPlane({1, 1}, FVector::ZeroVector, FVector::UpVector);
			auto C = Passive(E, Mu);
			auto S = BallTests::State();
			S.Position.Z = 10;
			S.LinearVelocity = FVector(400, 0, -2400);
			const double Before = Kinetic(S, C);
			FBallStepOutput O;
			TestTrue(TEXT("Passive material response"), Step(S, C, W, O));
			TestTrue(TEXT("Normal restitution analytic"), FMath::IsNearlyEqual(S.LinearVelocity.Z, E * 2400, 1.e-7));
			const double Jn = C.Mass * (1 + E) * 2400, Jt = FMath::Min(Mu * Jn, C.Mass * 400 / 3.5);
			TestTrue(TEXT("Coulomb tangent impulse analytic"),
					 FMath::IsNearlyEqual(S.LinearVelocity.X, 400 - Jt / C.Mass, 1.e-7));
			TestTrue(TEXT("Solid-sphere angular impulse"),
					 FMath::IsNearlyEqual(S.AngularVelocity.Y, C.Radius * Jt / C.GetSolidSphereInertia(), 1.e-7));
			TestTrue(TEXT("No unexplained kinetic energy gain"), Kinetic(S, C) <= Before + 1.e-5);
			if (E == 1 && Mu == 0)
			{
				TestTrue(TEXT("Elastic frictionless energy conserved"),
						 FMath::IsNearlyEqual(Kinetic(S, C), Before, 1.e-5));
			}
		}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallMaterials, "FourSquareVR.Ball.Collision.ConfigurableSurfaceMaterial",
								 CollisionFlags)
bool FBallMaterials::RunTest(const FString&)
{
	FBallSurfaceMaterial M;
	M.bOverride = true;
	M.Restitution = .25;
	M.Friction = 0;
	FRecordedBallCollisionWorld W;
	W.AddPlane({1, 1}, FVector::ZeroVector, FVector::UpVector, M);
	auto C = Passive(1, 1);
	auto S = BallTests::State();
	S.Position.Z = 10;
	S.LinearVelocity = FVector(100, 0, -2400);
	FBallStepOutput O;
	TestTrue(TEXT("Surface material step"), Step(S, C, W, O));
	TestTrue(TEXT("Geometric-mean coefficients"), S.LinearVelocity.Equals(FVector(100, 0, 1200), 1.e-7));
	C.Radius = 6;
	S = BallTests::State();
	S.Position.Z = 10;
	S.LinearVelocity.Z = -2400;
	TestTrue(TEXT("Designer radius changes TOI"), Step(S, C, W, O));
	TestTrue(TEXT("Radius-six exact time"), FMath::IsNearlyEqual(O.Events[0].Time.Fraction, .2, 1.e-9));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallPathological, "FourSquareVR.Ball.Collision.BoundedPathologicalFailure",
								 CollisionFlags)
bool FBallPathological::RunTest(const FString&)
{
	FRecordedBallCollisionWorld Narrow;
	Narrow.AddPlane({1, 1}, FVector(0, 0, 0), FVector::UpVector);
	Narrow.AddPlane({2, 1}, FVector(0, 0, 4), -FVector::UpVector);
	auto C = Passive();
	auto S = BallTests::State();
	S.Position.Z = 2;
	const auto Before = S;
	FBallSimulator Sim;
	FBallStepOutput O;
	TestTrue(TEXT("Impossible narrow gap fails explicitly"),
			 Sim.SimulateStep(S, BallTests::Inputs(S), C, Narrow, O) == EBallStepResult::PenetrationFailure);
	TestTrue(TEXT("Whole state preserved"), BallTests::ExactlyEqual(S, Before));
	TestTrue(TEXT("Bounded all retries"), O.Diagnostics.CollisionIterations <= 4u * (C.MaximumCollisionIterations + 1));
	TestEqual(TEXT("No partial event publication"), O.EventCount, 0);
	FRecordedBallCollisionWorld Corridor;
	Corridor.AddPlane({1, 1}, FVector::ZeroVector, FVector::ForwardVector);
	Corridor.AddPlane({2, 1}, FVector(10, 0, 0), -FVector::ForwardVector);
	S = BallTests::State();
	S.Position.X = 5;
	S.LinearVelocity.X = 120000;
	C.MaximumCollisionIterations = 2;
	const auto FastBefore = S;
	TestTrue(TEXT("Too many real bounces are not silently dropped"),
			 Sim.SimulateStep(S, BallTests::Inputs(S), C, Corridor, O) == EBallStepResult::CollisionLimit);
	TestTrue(TEXT("Atomic bounded collision budget failure"), BallTests::ExactlyEqual(S, FastBefore));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallCapacity, "FourSquareVR.Ball.Collision.ContactCapacityAndMalformedQueries",
								 CollisionFlags)
bool FBallCapacity::RunTest(const FString&)
{
	FRecordedBallCollisionWorld W;
	for (int32 I = 0; I < 9; ++I)
	{
		W.AddPlane({static_cast<uint64>(I + 1), 1}, FVector::ZeroVector, FVector::UpVector);
	}
	auto C = Passive();
	auto S = BallTests::State();
	S.Position.Z = 10;
	S.LinearVelocity.Z = -2400;
	const auto Before = S;
	FBallSimulator Sim;
	FBallStepOutput O;
	TestTrue(TEXT("Excess manifold is explicit capacity failure"),
			 Sim.SimulateStep(S, BallTests::Inputs(S), C, W, O) == EBallStepResult::ContactCapacity);
	TestTrue(TEXT("No discarded normal or partial step"), BallTests::ExactlyEqual(S, Before));
	class FBadWorld final : public IBallCollisionWorld
	{
	  public:
		uint32 GetRevision() const override
		{
			return 1;
		}
		EBallQueryResult SweepSphere(const FBallCollisionQuery&, FBallContact& Out) const override
		{
			Out.Normal = FVector::ZeroVector;
			Out.Collider = {1, 1};
			return EBallQueryResult::Contact;
		}
	} Bad;
	TestTrue(TEXT("Invalid normal from adapter rejected"),
			 Sim.SimulateStep(S, BallTests::Inputs(S), C, Bad, O) == EBallStepResult::QueryFailure);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallTieTolerance, "FourSquareVR.Ball.Collision.EarliestTOIToleranceOrdering",
								 CollisionFlags)
bool FBallTieTolerance::RunTest(const FString&)
{
	for (int32 Order = 0; Order < 6; ++Order)
	{
		static const int32 Permutations[6][3] = {{0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};
		FBallCollisionQuery Q;
		Q.TOIFractionTolerance = .001;
		FBallContactBatch Batch;
		for (int32 I = 0; I < 3; ++I)
		{
			int32 J = Permutations[Order][I];
			FBallContact Hit;
			Hit.Collider = {static_cast<uint64>(J + 1), 1};
			Hit.TimeOfImpact = .5 + J * .00075;
			TestTrue(TEXT("Valid tied contact"), Batch.Add(Hit, Q));
		}
		Batch.Sort();
		TestEqual(TEXT("Only within tolerance of true earliest"), Batch.Count, 2);
		TestEqual(TEXT("Stable first ID"), Batch.Contacts[0].Collider.Value, uint64(1));
	}
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallAcuteCorner, "FourSquareVR.Ball.Collision.AcuteCornerManifold", CollisionFlags)
bool FBallAcuteCorner::RunTest(const FString&)
{
	const FVector A(1, 0, 0), B = FVector(-.99, .14106735979665894, 0).GetSafeNormal();
	for (double Mu : {0.0, .35})
	{
		FRecordedBallCollisionWorld W;
		W.AddPlane({1, 1}, -A * 3, A);
		W.AddPlane({2, 1}, -B * 3, B);
		auto C = Passive(1, Mu);
		auto S = BallTests::State();
		S.LinearVelocity = FVector(0, -1000, 0);
		const double Before = Kinetic(S, C);
		FBallStepOutput O;
		TestTrue(TEXT("Acute simultaneous constraints resolve"), Step(S, C, W, O));
		TestTrue(TEXT("All normals non-closing"), FVector::DotProduct(S.LinearVelocity, A) >= -1.e-4 &&
													  FVector::DotProduct(S.LinearVelocity, B) >= -1.e-4);
		TestTrue(TEXT("Acute contact remains passive"), Kinetic(S, C) <= Before + 1.e-5);
		if (Mu == 0)
		{
			TestTrue(TEXT("Symmetric elastic wedge response"), S.LinearVelocity.Equals(FVector(0, 1000, 0), 1.e-5));
		}
	}
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallGentleSlope, "FourSquareVR.Ball.Collision.RestingSphereMustRollOnSlope",
								 CollisionFlags)
bool FBallGentleSlope::RunTest(const FString&)
{
	const FVector N = FVector(.1, 0, .99498743710662).GetSafeNormal();
	FRecordedBallCollisionWorld W;
	W.AddPlane({1, 1}, FVector::ZeroVector, N);
	FBallPhysicsConfig C;
	auto S = BallTests::State();
	S.Position = N * 3.02;
	S.Activity = EBallActivityState::Resting;
	S.bHasSupport = true;
	S.SupportCollider = {1, 1};
	S.SupportNormal = N;
	S.SleepDwellTime = 1;
	FBallStepOutput O;
	for (int32 I = 0; I < 120; ++I)
	{
		if (!Step(S, C, W, O))
		{
			AddError(TEXT("Gentle rolling slope failed"));
			return false;
		}
	}
	TestTrue(TEXT("Static friction does not freeze a free sphere"),
			 S.Activity == EBallActivityState::Active && S.LinearVelocity.Size() > 50 && S.AngularVelocity.Size() > 10);
	return true;
}
#endif
