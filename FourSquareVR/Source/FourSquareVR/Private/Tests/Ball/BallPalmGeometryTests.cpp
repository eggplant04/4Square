#include "Misc/AutomationTest.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "BallPalmTestFixtures.h"
#include "Ball/BallHandPoseHistory.h"
#include "Ball/BallCollisionSolver.h"
#include <limits>
namespace
{
constexpr auto PalmGeometryFlags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmCalibration, "FourSquareVR.Ball.Palm.Geometry.ControllerCalibration",
								 PalmGeometryFlags)
bool FPalmCalibration::RunTest(const FString &)
{
	FBallPalmSettings L, R;
	L.CenterOffset = FVector(2, 3, 4);
	L.RotationOffset = FRotator(30, 20, 10);
	R.CenterOffset = FVector(-2, 3, 4);
	R.RotationOffset = FRotator(30, -20, -10);
	const FQuat Q(FVector::UpVector, PI / 2);
	const FVector P(10, 20, 30);
	const auto WL = L.WorldTransform(P, Q), WR = R.WorldTransform(P, Q);
	TestTrue(TEXT("Position offset is controller local, not rotated twice by calibration"),
			 WL.GetLocation().Equals(FVector(7, 22, 34), 1.e-6));
	TestTrue(TEXT("Rotation composition"), WL.GetRotation().Equals(Q * L.RotationOffset.Quaternion(), 1.e-10));
	TestTrue(TEXT("Independent mirrored calibration"),
			 WR.GetLocation().Equals(FVector(7, 18, 34), 1.e-6) && !WR.GetRotation().Equals(WL.GetRotation(), 1.e-5));
	TestTrue(TEXT("Thin nonspherical default"),
			 L.BoxDimensions.Z < L.BoxDimensions.X && L.BoxDimensions.X < L.BoxDimensions.Y);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmFaces, "FourSquareVR.Ball.Palm.Geometry.AllSixFacesAndBroadFaceConsistency",
								 PalmGeometryFlags)
bool FPalmFaces::RunTest(const FString &)
{
	auto F = PalmTests::Frame();
	FBallContact Hit;
	for (int32 Axis = 0; Axis < 3; ++Axis)
		for (int32 Sign : {-1, 1})
		{
			FVector N = FVector::ZeroVector;
			N[Axis] = Sign;
			const double Extent = F.Colliders[0].HalfExtents[Axis];
			auto Q = PalmTests::Query(N * (Extent + 3), FVector::ZeroVector);
			TestTrue(TEXT("Face swept contact"),
					 FBallHandCollisionSolver::Query(F, F.Colliders[0], Q, Hit) == EBallQueryResult::Contact);
			TestEqual(TEXT("Analytic TOI"), Hit.TimeOfImpact, 2 / (Extent + 3), 1.e-9);
			TestTrue(TEXT("Actual outward face normal"), Hit.Normal.Equals(N, 1.e-10));
			TestTrue(TEXT("Contact on palm outer face"), Hit.Position.Equals(N * Extent, 1.e-9));
		}
	for (double X : {-2.5, 0.0, 2.5})
	{
		auto Q = PalmTests::Query(FVector(X, 1, 5), FVector(X, 1, -5));
		FBallHandCollisionSolver::Query(F, F.Colliders[0], Q, Hit);
		TestTrue(TEXT("Broad-face position does not create radial deflection"),
				 Hit.Normal.Equals(FVector::UpVector, 1.e-10));
		TestEqual(TEXT("Same face timing"), Hit.TimeOfImpact, .3, 1.e-10);
	}
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmCorners, "FourSquareVR.Ball.Palm.Geometry.RoundedEdgesCornersAndInside",
								 PalmGeometryFlags)
bool FPalmCorners::RunTest(const FString &)
{
	const FVector H(4, 6, 1), Core = H - FVector(.5);
	for (const FVector N : {FVector(1, 0, 1).GetSafeNormal(), FVector(1, 1, 1).GetSafeNormal()})
	{
		FVector Closest = Core;
		if (N.Y == 0)
			Closest.Y = 0;
		const auto D = FBallPalmGeometry::Distance(Closest + N * 1.5, 1, FVector::ZeroVector, FQuat::Identity, H, .5);
		TestEqual(TEXT("Rounded surface gap"), D.Separation, 0.0, 1.e-10);
		TestTrue(TEXT("Geometric edge/corner normal"), D.Normal.Equals(N, 1.e-10));
		TestTrue(TEXT("Palm and ball points agree"), D.Point.Equals(Closest + N * .5, 1.e-10));
	}
	const auto Inside =
		FBallPalmGeometry::Distance(FVector::ZeroVector, 1, FVector::ZeroVector, FQuat::Identity, H, .5);
	TestEqual(TEXT("Inside nearest exit depth"), Inside.Separation, -2.0, 1.e-10);
	TestTrue(TEXT("Inside nearest thin face"), Inside.Normal.Equals(FVector::UpVector));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmTranslated, "FourSquareVR.Ball.Palm.CCD.TranslatingPalmAnalyticResponse",
								 PalmGeometryFlags)
bool FPalmTranslated::RunTest(const FString &)
{
	auto F = PalmTests::Frame(FVector(0, 0, -5), FVector::ZeroVector);
	auto S = BallTests::State();
	FBallStepOutput O;
	TestTrue(TEXT("Step"), PalmTests::Step(F, S, PalmTests::Config(), O) == EBallStepResult::Success);
	TestEqual(TEXT("One event"), O.EventCount, 1);
	if (!O.EventCount)
		return false;
	TestEqual(TEXT("Analytic TOI"), O.Events[0].Time.Fraction, .6, 1.e-9);
	TestEqual(TEXT("Kinematic restitution"), S.LinearVelocity.Z, 1080.0, 1.e-8);
	TestEqual(TEXT("Remaining time consumed"), S.Position.Z, 3.62, 1.e-8);
	TestTrue(TEXT("Broad-face normal"), O.Events[0].Normal.Equals(FVector::UpVector, 1.e-10));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmBallMoving, "FourSquareVR.Ball.Palm.CCD.MovingBallStationaryBox",
								 PalmGeometryFlags)
bool FPalmBallMoving::RunTest(const FString &)
{
	auto S = BallTests::State();
	S.Position.Z = 5;
	S.LinearVelocity.Z = -1200;
	FBallStepOutput O;
	TestTrue(TEXT("Step"), PalmTests::Step(PalmTests::Frame(), S, PalmTests::Config(), O) == EBallStepResult::Success);
	if (!TestEqual(TEXT("One event"), O.EventCount, 1))
		return false;
	TestEqual(TEXT("TOI"), O.Events[0].Time.Fraction, .3, 1.e-9);
	TestEqual(TEXT("Vout"), S.LinearVelocity.Z, 960.0, 1.e-8);
	TestEqual(TEXT("Endpoint"), S.Position.Z, 7.62, 1.e-8);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmWristCCD, "FourSquareVR.Ball.Palm.CCD.RotatingEdgeClearEndpointsAnalyticTOI",
								 PalmGeometryFlags)
bool FPalmWristCCD::RunTest(const FString &)
{
	auto F = PalmTests::Wrist();
	auto Q = PalmTests::Query(FVector(5, 0, 0), FVector(5, 0, 0), .5);
	FBallContact H;
	TestTrue(TEXT("Both endpoints clear"),
			 FBallHandCollisionSolver::Separation(F, F.Colliders[0], 0, Q.Start, .5) > 0 &&
				 FBallHandCollisionSolver::Separation(F, F.Colliders[0], 1, Q.Start, .5) > 0);
	FBallMotionCCDStatistics Stats;
	Q.MotionStatistics = &Stats;
	TestTrue(TEXT("Rotation alone strikes between clear endpoints"),
			 FBallHandCollisionSolver::Query(F, F.Colliders[0], Q, H) == EBallQueryResult::Contact);
	const double Angle = -FMath::Asin(.2), Expected = (Angle + .3) / .6;
	const FVector N(-FMath::Sin(Angle), FMath::Cos(Angle), 0);
	TestEqual(TEXT("Earliest rotating side TOI"), H.TimeOfImpact, Expected, 3.e-6);
	TestTrue(TEXT("Rotated geometric normal"), H.Normal.Equals(N, 3.e-6));
	TestTrue(TEXT("Palm actual surface"), H.Position.Equals(Q.Start - N * .5, 3.e-6));
	TestTrue(TEXT("Wrist velocity at contact"),
			 H.SurfaceVelocity.Equals(FVector::CrossProduct(FVector(0, 0, 72), H.Position), 1.e-8));
	TestTrue(TEXT("Bounded converged work"), Stats.Iterations > 1 && Stats.Iterations <= 256 && Stats.Unresolved == 0);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmRapidCCD, "FourSquareVR.Ball.Palm.CCD.RapidRotationOpposingMotionAndTunnelling",
								 PalmGeometryFlags)
bool FPalmRapidCCD::RunTest(const FString &)
{
	for (double Angle : {.6, 1.5, 2.7})
	{
		auto F = PalmTests::Wrist();
		F.Segments[0].StartOrientation = FQuat(FVector::UpVector, -Angle / 2);
		F.Segments[0].EndOrientation = FQuat(FVector::UpVector, Angle / 2);
		FBallContact H;
		auto Q = PalmTests::Query(FVector(5, 0, 0), FVector(5, 0, 0), .5);
		TestTrue(TEXT("High angular sweep"),
				 FBallHandCollisionSolver::Query(F, F.Colliders[0], Q, H) == EBallQueryResult::Contact);
		TestEqual(TEXT("Analytic rapid-rotation TOI"), H.TimeOfImpact, (Angle / 2 - FMath::Asin(.2)) / Angle, 3.e-6);
	}
	auto F = PalmTests::Frame(FVector(0, 0, -100), FVector(0, 0, 100), FQuat::Identity, FQuat(FVector::UpVector, 1));
	FBallContact H;
	auto Q = PalmTests::Query(FVector(0, 0, 100), FVector(0, 0, -100));
	TestTrue(TEXT("Combined rapid translation/rotation"),
			 FBallHandCollisionSolver::Query(F, F.Colliders[0], Q, H) == EBallQueryResult::Contact);
	TestEqual(TEXT("Opposing thin-face analytic TOI"), H.TimeOfImpact, 198.0 / 400, 2.e-6);
	TestTrue(TEXT("Face is not artificial radial normal"), H.Normal.Equals(FVector::UpVector, 1.e-8));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmBudget, "FourSquareVR.Ball.Palm.CCD.UnresolvedIsAtomicFailure", PalmGeometryFlags)
bool FPalmBudget::RunTest(const FString &)
{
	auto F = PalmTests::Wrist();
	F.Colliders[0].MaximumCCDIterations = 1;
	FBallContact H;
	auto Q = PalmTests::Query(FVector(5, 0, 0), FVector(5, 0, 0), .5);
	TestTrue(TEXT("Budget exhaustion is not clear"),
			 FBallHandCollisionSolver::Query(F, F.Colliders[0], Q, H) == EBallQueryResult::Unresolved);
	auto S = BallTests::State();
	S.Position = Q.Start;
	const auto Before = S;
	auto C = PalmTests::Config();
	C.Radius = .5;
	FBallStepOutput O;
	TestTrue(TEXT("Simulation propagates failure"), PalmTests::Step(F, S, C, O) == EBallStepResult::QueryFailure);
	TestTrue(TEXT("No partially advanced branch"), BallTests::ExactlyEqual(S, Before));
	TestEqual(TEXT("No published events"), O.EventCount, 0);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmGrazing, "FourSquareVR.Ball.Palm.CCD.GrazingAndRotatingParallelClear",
								 PalmGeometryFlags)
bool FPalmGrazing::RunTest(const FString &)
{
	auto F = PalmTests::Frame(FVector::ZeroVector, FVector::ZeroVector, FQuat::Identity, FQuat(FVector::UpVector, 2.7));
	FBallContact H;
	auto Q = PalmTests::Query(FVector(0, 0, 2.001), FVector(0, 0, 2.001));
	FBallMotionCCDStatistics Stats;
	Q.MotionStatistics = &Stats;
	TestTrue(TEXT("Rapid face-parallel rotation certified clear"),
			 FBallHandCollisionSolver::Query(F, F.Colliders[0], Q, H) == EBallQueryResult::Clear);
	TestTrue(TEXT("Directed bound avoids hundreds of unnecessary advances"), Stats.Iterations <= 2);
	F = PalmTests::Frame();
	Q = PalmTests::Query(FVector(-10, 0, 2), FVector(10, 0, 2));
	TestTrue(TEXT("Exact tangency without artificial impulse"),
			 FBallHandCollisionSolver::Query(F, F.Colliders[0], Q, H) == EBallQueryResult::Clear);
	Q.Start.Z = Q.End.Z = 1.999;
	TestTrue(TEXT("Near tangent catches rounded edge"),
			 FBallHandCollisionSolver::Query(F, F.Colliders[0], Q, H) == EBallQueryResult::Contact);
	TestTrue(TEXT("Rounded glancing normal"), H.Normal.X < 0 && H.Normal.Z > .99);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmWristImpulse, "FourSquareVR.Ball.Palm.Response.WristOnlyNormalImpulse",
								 PalmGeometryFlags)
bool FPalmWristImpulse::RunTest(const FString &)
{
	auto F = PalmTests::Wrist();
	auto S = BallTests::State();
	S.Position = FVector(5, 0, 0);
	auto C = PalmTests::Config();
	C.Radius = .5;
	FBallStepOutput O;
	TestTrue(TEXT("Wrist step"), PalmTests::Step(F, S, C, O) == EBallStepResult::Success);
	if (!TestTrue(TEXT("Wrist strike event"), O.EventCount > 0))
		return false;
	const double A = -FMath::Asin(.2);
	const FVector N(-FMath::Sin(A), FMath::Cos(A), 0);
	const FVector Point = FVector(5, 0, 0) - N * .5, V = FVector::CrossProduct(FVector(0, 0, 72), Point);
	const FVector Expected = N * (1.8 * FVector::DotProduct(V, N));
	TestTrue(TEXT("Angular surface velocity drives correct momentum"),
			 O.Events[0].OutgoingVelocity.Equals(Expected, .002));
	TestTrue(TEXT("Zero friction produces no spin"), O.Events[0].OutgoingAngularVelocity.IsNearlyZero(1.e-9));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmAngleResponse,
								 "FourSquareVR.Ball.Palm.Response.AngledWristRedirectsByGeometricNormal",
								 PalmGeometryFlags)
bool FPalmAngleResponse::RunTest(const FString &)
{
	const FQuat Q(FVector::YAxisVector, PI / 6);
	const FVector N = Q.RotateVector(FVector::UpVector);
	auto F = PalmTests::Frame(FVector::ZeroVector, FVector::ZeroVector, Q, Q);
	auto S = BallTests::State();
	S.Position = N * 5;
	S.LinearVelocity = FVector(0, 0, -1200);
	FBallStepOutput O;
	const FVector Expected = S.LinearVelocity - N * (1.8 * FVector::DotProduct(S.LinearVelocity, N));
	TestTrue(TEXT("Step"), PalmTests::Step(F, S, PalmTests::Config(), O) == EBallStepResult::Success);
	if (!TestEqual(TEXT("One event"), O.EventCount, 1))
		return false;
	TestTrue(TEXT("Geometry controls direction"),
			 O.Events[0].Normal.Equals(N, 1.e-8) && S.LinearVelocity.Equals(Expected, 1.e-6));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmSpin, "FourSquareVR.Ball.Palm.Response.GlancingFrictionSpinAndPassiveEnergy",
								 PalmGeometryFlags)
bool FPalmSpin::RunTest(const FString &)
{
	for (double Mu : {0.0, .35})
	{
		auto F = PalmTests::Frame();
		F.Colliders[0].Material.Friction = Mu;
		auto C = PalmTests::Config();
		C.SurfaceFriction = Mu;
		auto S = BallTests::State();
		S.Position = FVector(0, 0, 5);
		S.LinearVelocity = FVector(300, 0, -1200);
		const double E = (.5 * C.Mass * S.LinearVelocity.SizeSquared() +
						  .5 * C.GetSolidSphereInertia() * S.AngularVelocity.SizeSquared());
		FBallStepOutput O;
		TestTrue(TEXT("Glancing response"), PalmTests::Step(F, S, C, O) == EBallStepResult::Success);
		TestEqual(TEXT("Normal restitution"), S.LinearVelocity.Z, 960.0, 1.e-7);
		TestEqual(TEXT("Tangential momentum"), S.LinearVelocity.X, Mu == 0 ? 300.0 : 300.0 * 5 / 7, 1.e-7);
		TestEqual(TEXT("Contact torque spin"), S.AngularVelocity.Y, Mu == 0 ? 0.0 : 300.0 * 5 / 7, 1.e-7);
		TestTrue(TEXT("Passive energy dissipates"),
				 (.5 * C.Mass * S.LinearVelocity.SizeSquared() +
				  .5 * C.GetSolidSphereInertia() * S.AngularVelocity.SizeSquared()) <= E + 1.e-7);
	}
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmProfileValidity,
								 "FourSquareVR.Ball.Palm.Geometry.InvalidSettingsAndFaceMaterials", PalmGeometryFlags)
bool FPalmProfileValidity::RunTest(const FString &)
{
	FBallPalmSettings P;
	P.RoundRadius = 100;
	TestFalse(TEXT("Impossible rounding rejected"), P.IsValid());
	P = {};
	P.BoxDimensions.Z = 0;
	TestFalse(TEXT("Zero thickness rejected"), P.IsValid());
	P = {};
	P.RotationOffset.Yaw = std::numeric_limits<double>::infinity();
	TestFalse(TEXT("Nonfinite calibration rejected"), P.IsValid());
	auto F = PalmTests::Frame();
	auto &M = F.Colliders[0].FaceMaterials;
	M.bEnabled = true;
	M.Front.bOverride = M.Back.bOverride = M.Side.bOverride = true;
	M.Front.Restitution = .2;
	M.Back.Restitution = .4;
	M.Side.Restitution = .6;
	FBallContact H;
	for (const FVector N : {FVector::UpVector, -FVector::UpVector, FVector::ForwardVector})
	{
		auto Q = PalmTests::Query(N * 10, FVector::ZeroVector);
		TestTrue(TEXT("Front/back/side remain physically enabled"),
				 FBallHandCollisionSolver::Query(F, F.Colliders[0], Q, H) == EBallQueryResult::Contact);
		TestEqual(TEXT("Geometric face chooses material"), H.Material.Restitution, N.Z > 0 ? .2 : N.Z < 0 ? .4 : .6);
	}
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmNearAngularTangent,
								 "FourSquareVR.Ball.Palm.CCD.NearAngularGrazingClearanceAndLifetime", PalmGeometryFlags)
bool FPalmNearAngularTangent::RunTest(const FString &)
{
	auto F = PalmTests::Frame(FVector::ZeroVector, FVector::ZeroVector, FQuat(FVector::YAxisVector, -1.2),
							  FQuat(FVector::YAxisVector, 1.2));
	const double X = FMath::Sqrt(3.5 * 3.5 + .5 * .5) + 1.5 + .0001;
	auto Q = PalmTests::Query(FVector(X, 0, 0), FVector(X, 0, 0));
	FBallContact H;
	FBallMotionCCDStatistics Stats;
	Q.MotionStatistics = &Stats;
	TestTrue(TEXT("Grazing past maximum rounded corner extent stays clear"),
			 FBallHandCollisionSolver::Query(F, F.Colliders[0], Q, H) == EBallQueryResult::Clear);
	AddInfo(FString::Printf(TEXT("PC_ANGULAR_GRAZE Iterations=%u Unresolved=%u"), Stats.MaximumIterations,
							Stats.Unresolved));
	for (auto Shape : {EBallMotionShape::Sphere, EBallMotionShape::RoundedBox})
	{
		F = PalmTests::Frame();
		F.Colliders[0].Shape = Shape;
		F.Segments[0].EndFraction = .25;
		Q = PalmTests::Query(FVector(0, 0, 2), FVector(0, 0, 2));
		Q.Time.Fraction = .25;
		TestTrue(TEXT("Expired endpoint is not an outgoing trajectory"),
				 FBallHandCollisionSolver::Query(F, F.Colliders[0], Q, H) == EBallQueryResult::Clear);
	}
	return true;
}
#endif
