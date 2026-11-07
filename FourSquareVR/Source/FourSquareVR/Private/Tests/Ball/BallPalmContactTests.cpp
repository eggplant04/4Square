#include "Misc/AutomationTest.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "BallPalmTestFixtures.h"
#include "Ball/BallStateCodec.h"
namespace
{
constexpr auto PalmContactFlags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;
void Established(FBallState &S, const FBallColliderId &Id)
{
	S.MotionPairCount = 1;
	S.MotionPairs[0].Collider = Id;
	S.MotionPairs[0].Episode = 1;
	S.MotionPairs[0].bArmed = false;
}
} // namespace
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmInitialOverlap, "FourSquareVR.Ball.Palm.Contact.UnknownOverlapQuarantine",
								 PalmContactFlags)
bool FPalmInitialOverlap::RunTest(const FString &)
{
	auto F = PalmTests::Frame();
	auto S = BallTests::State();
	FBallStepOutput O;
	TestTrue(TEXT("Cold overlap safe"), PalmTests::Step(F, S, PalmTests::Config(), O) == EBallStepResult::Success);
	TestTrue(TEXT("No invented MTD or impulse"), S.Position.IsZero() && S.LinearVelocity.IsZero() && O.EventCount == 0);
	TestTrue(TEXT("Pair quarantine restored state"),
			 S.MotionPairs[0].bBlocked && S.MotionPairs[0].BlockReason == EBallMotionBlockReason::InitialOverlap);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmPush, "FourSquareVR.Ball.Palm.Contact.PersistentPushAndRestoreNoDuplicateImpact",
								 PalmContactFlags)
bool FPalmPush::RunTest(const FString &)
{
	auto F = PalmTests::Frame(FVector::ZeroVector, FVector(0, 0, .1));
	auto S = BallTests::State();
	S.Position.Z = 2.02;
	Established(S, F.Colliders[0].Id);
	FBallStepOutput O;
	TestTrue(TEXT("Persistent push"), PalmTests::Step(F, S, PalmTests::Config(), O) == EBallStepResult::Success);
	TestEqual(TEXT("Inelastic prescribed surface speed"), S.LinearVelocity.Z, 12.0, 1.e-7);
	TestEqual(TEXT("Remaining push and skin"), S.Position.Z, 2.12, 1.e-7);
	TestEqual(TEXT("No artificial strike"), O.EventCount, 0);
	TArray<uint8> Bytes;
	FBallState Restored;
	FBallStateCodec::Save(S, Bytes);
	FBallStateCodec::Load(Bytes, Restored);
	F.Segments[0].Start.Z = .1;
	F.Segments[0].End.Z = .2;
	auto Reference = S;
	TestTrue(TEXT("Continue after restore"),
			 PalmTests::Step(F, Restored, PalmTests::Config(), O) == EBallStepResult::Success && O.EventCount == 0);
	FBallStepOutput Other;
	PalmTests::Step(F, Reference, PalmTests::Config(), Other);
	TestTrue(TEXT("Complete contact-state replay"), BallTests::ExactlyEqual(Restored, Reference));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmRest, "FourSquareVR.Ball.Palm.Contact.ExtendedBroadFaceRest", PalmContactFlags)
bool FPalmRest::RunTest(const FString &)
{
	auto F = PalmTests::Frame();
	auto S = BallTests::State();
	S.Position.Z = 2.02;
	Established(S, F.Colliders[0].Id);
	auto C = PalmTests::Config();
	C.Gravity.Z = -980.665;
	FBallStepOutput O;
	int32 Events = 0;
	for (int32 I = 0; I < 3600; ++I)
	{
		if (PalmTests::Step(F, S, C, O) != EBallStepResult::Success)
		{
			AddError(FString::Printf(TEXT("Rest failed frame %d result %d"), I, int32(O.Result)));
			return false;
		}
		Events += O.EventCount;
	}
	TestEqual(TEXT("Stable thirty-second support height"), S.Position.Z, 2.02, 1.e-5);
	TestTrue(TEXT("No energy or jitter accumulation"),
			 S.LinearVelocity.IsNearlyZero(1.e-6) && S.AngularVelocity.IsNearlyZero(1.e-6));
	TestEqual(TEXT("No duplicate impacts"), Events, 0);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmRotatingContact,
								 "FourSquareVR.Ball.Palm.Contact.RotationDuringSustainedFaceContact", PalmContactFlags)
bool FPalmRotatingContact::RunTest(const FString &)
{
	auto F = PalmTests::Frame();
	F.Colliders[0].Material.Friction = .35;
	auto S = BallTests::State();
	S.Position = FVector(2, 0, 2.02);
	Established(S, F.Colliders[0].Id);
	auto C = PalmTests::Config();
	C.SurfaceFriction = .35;
	C.Gravity.Z = -980.665;
	FBallStepOutput O;
	for (int32 I = 0; I < 20; ++I)
	{
		F.Segments[0].StartOrientation = FQuat(FVector::UpVector, I * .1);
		F.Segments[0].EndOrientation = FQuat(FVector::UpVector, (I + 1) * .1);
		if (PalmTests::Step(F, S, C, O) != EBallStepResult::Success)
		{
			AddError(FString::Printf(TEXT("Sustained rotation frame %d failed %d"), I, int32(O.Result)));
			return false;
		}
		TestEqual(TEXT("No repeated restitution event while rubbing"), O.EventCount, 0);
		TestTrue(TEXT("Finite motor-driven response"),
				 S.IsValid() && S.LinearVelocity.Size() < 100 && S.AngularVelocity.Size() < 100);
	}
	TestTrue(TEXT("Surface rotation generates tangential motion"), S.LinearVelocity.Size() > 1);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmBoth, "FourSquareVR.Ball.Palm.Contact.SimultaneousPalmsAndPermutation",
								 PalmContactFlags)
bool FPalmBoth::RunTest(const FString &)
{
	auto A = PalmTests::Frame(FVector(0, 0, -5), FVector::ZeroVector);
	auto B = PalmTests::Frame(FVector(-5, 0, 0), FVector::ZeroVector, FQuat(FVector::YAxisVector, UE_DOUBLE_PI / 2),
							  FQuat(FVector::YAxisVector, UE_DOUBLE_PI / 2));
	B.Colliders[0].Id = BallMotionLimits::HandId(1, EBallHandSide::Right, 1);
	B.Colliders[0].Side = EBallHandSide::Right;
	auto F = A;
	F.Add(B.Colliders[0], MakeArrayView(B.Segments, 1));
	auto G = B;
	G.Add(A.Colliders[0], MakeArrayView(A.Segments, 1));
	auto S = BallTests::State(), T = S;
	FBallStepOutput O, P;
	TestTrue(TEXT("Two constraints jointly solved"),
			 PalmTests::Step(F, S, PalmTests::Config(), O) == EBallStepResult::Success);
	TestTrue(TEXT("Permutation jointly solved"),
			 PalmTests::Step(G, T, PalmTests::Config(), P) == EBallStepResult::Success);
	TestEqual(TEXT("Two legitimate contacts"), O.EventCount, 2);
	TestTrue(TEXT("Joint restitution"), S.LinearVelocity.Equals(FVector(1080, 0, 1080), 1.e-5));
	TestTrue(TEXT("Packet/input order cannot change full state"), BallTests::ExactlyEqual(S, T));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmSqueeze, "FourSquareVR.Ball.Palm.Contact.ImpossibleOpposingSqueezeStablePolicy",
								 PalmContactFlags)
bool FPalmSqueeze::RunTest(const FString &)
{
	auto F = PalmTests::Frame(FVector(0, 0, -5), FVector::ZeroVector);
	auto B = PalmTests::Frame(FVector(0, 0, 5), FVector::ZeroVector);
	B.Colliders[0].Id = BallMotionLimits::HandId(1, EBallHandSide::Right, 1);
	B.Colliders[0].Side = EBallHandSide::Right;
	F.Add(B.Colliders[0], MakeArrayView(B.Segments, 1));
	auto S = BallTests::State();
	FBallStepOutput O;
	TestTrue(TEXT("Impossible squeeze has bounded policy"),
			 PalmTests::Step(F, S, PalmTests::Config(), O) == EBallStepResult::Success);
	TestEqual(TEXT("Single admitted strike"), O.EventCount, 1);
	TestTrue(TEXT("Explicit conflict rejection"), O.Diagnostics.RejectedMotionContacts > 0);
	TestEqual(TEXT("No opposing infinite impulse"), S.LinearVelocity.Z, 1080.0, 1.e-6);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmWorldRemainder, "FourSquareVR.Ball.Palm.Contact.PalmThenCeilingRemainingTime",
								 PalmContactFlags)
bool FPalmWorldRemainder::RunTest(const FString &)
{
	auto F = PalmTests::Frame(FVector(0, 0, -10), FVector(0, 0, 10));
	auto S = BallTests::State();
	FBallStepOutput O;
	FRecordedBallCollisionWorld W;
	W.AddPlane({1, 1}, FVector(0, 0, 20), -FVector::UpVector);
	TestTrue(TEXT("Hand/world remainder"),
			 PalmTests::Step(F, S, PalmTests::Config(), O, &W) == EBallStepResult::Success);
	if (!TestEqual(TEXT("Both impacts"), O.EventCount, 2))
		return false;
	TestTrue(TEXT("Physical event ordering"), O.Events[0].Category == EBallContactCategory::Hand &&
												  O.Events[1].Category != EBallContactCategory::Hand &&
												  O.Events[1].Time.Fraction > O.Events[0].Time.Fraction);
	TestEqual(TEXT("Ceiling restitution"), S.LinearVelocity.Z, -3456.0, 1.e-6);
	TestEqual(TEXT("Analytic double-impact endpoint"), S.Position.Z, 16.884, 1.e-6);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmFloorContact, "FourSquareVR.Ball.Palm.Contact.HandAndFloorJointManifold",
								 PalmContactFlags)
bool FPalmFloorContact::RunTest(const FString &)
{
	const FQuat R(FVector::YAxisVector, UE_DOUBLE_PI / 2);
	auto F = PalmTests::Frame(FVector(-5, 0, 1.02), FVector(0, 0, 1.02), R, R);
	auto S = BallTests::State();
	S.Position.Z = 1.02;
	FBallStepOutput O;
	FRecordedBallCollisionWorld W;
	W.AddPlane({1, 1}, FVector::ZeroVector, FVector::UpVector);
	TestTrue(TEXT("Joint floor/palm solve"),
			 PalmTests::Step(F, S, PalmTests::Config(), O, &W) == EBallStepResult::Success);
	TestEqual(TEXT("Floor height maintained"), S.Position.Z, 1.02, 1.e-7);
	TestEqual(TEXT("Sideways hand response"), S.LinearVelocity.X, 1080.0, 1.e-6);
	return true;
}
#endif
