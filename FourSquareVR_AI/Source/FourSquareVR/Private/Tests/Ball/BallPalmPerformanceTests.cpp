#include "Misc/AutomationTest.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "BallPalmTestFixtures.h"
#include "Ball/BallReplayRunner.h"
#include "Ball/BallTestCourt.h"
#include "Ball/UnrealBallCollisionWorld.h"
#include "Ball/VRHandCollisionComponent.h"
#include "Ball/HandballBall.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"
namespace
{
constexpr auto PalmPerformanceFlags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;
FBallMotionFrame ProfilePalms(uint64 Frame, bool Box, bool Stress = false)
{
	FBallMotionFrame F;
	F.Epoch = BallTests::State().Identity.Epoch;
	F.Frame = Frame;
	for (int32 K = 0; K < 12; ++K)
	{
		FBallColliderMotion C;
		C.Id = K < 8 ? BallMotionLimits::HandId(K / 2 + 1, EBallHandSide(K % 2), 1)
					 : FBallColliderId{BallMotionLimits::KinematicNamespace | uint64(K - 7), 1};
		C.Participant = K < 8 ? K / 2 + 1 : 0;
		C.Side = EBallHandSide(K % 2);
		C.OriginRevision = 1;
		C.Category = K < 8 ? EBallContactCategory::Hand : EBallContactCategory::Kinematic;
		C.Shape = Box && K < 8 ? EBallMotionShape::RoundedBox : EBallMotionShape::Sphere;
		C.HalfExtents = FVector(4.5, 5.5, 1.25);
		C.RoundRadius = .75;
		C.Material.bOverride = true;
		C.Material.Friction = 0;
		FBallMotionSegment Segments[8];
		const int32 N = Stress ? 8 : 1;
		for (int32 J = 0; J < N; ++J)
		{
			auto &S = Segments[J];
			S.StartFraction = double(J) / N + (J ? K * .0001 : 0);
			S.EndFraction = double(J + 1) / N + (J + 1 < N ? K * .0001 : 0);
			const double A = (Frame + S.StartFraction) / 120.0, B = (Frame + S.EndFraction) / 120.0;
			S.Start = FVector(60 + 20 * FMath::Sin(2 * UE_DOUBLE_PI * A) + K * 30, K == 0 ? 0 : 100 + K * 20, 500);
			S.End = FVector(60 + 20 * FMath::Sin(2 * UE_DOUBLE_PI * B) + K * 30, K == 0 ? 0 : 100 + K * 20, 500);
			S.StartOrientation =
				FQuat(FVector::UpVector, .1 * FMath::Sin(3 * A)) * FQuat(FVector::YAxisVector, UE_DOUBLE_PI / 2);
			S.EndOrientation =
				FQuat(FVector::UpVector, .1 * FMath::Sin(3 * B)) * FQuat(FVector::YAxisVector, UE_DOUBLE_PI / 2);
			S.FirstSample = uint32(Frame + 1);
			S.SecondSample = uint32(Frame + 2);
		}
		check(F.Add(C, MakeArrayView(Segments, N)));
	}
	return F;
}
} // namespace
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmAdversarialOracle,
								 "FourSquareVR.Ball.Palm.CCD.SeededDenseOracleAndBoundedKernelProfile",
								 PalmPerformanceFlags)
bool FPalmAdversarialOracle::RunTest(const FString &)
{
	FRandomStream Random(410120);
	uint32 Hits = 0, Clear = 0, Unresolved = 0;
	FBallMotionCCDStatistics Stats;
	TArray<double> Times;
	Times.Reserve(1024);
	for (int32 I = 0; I < 1024; ++I)
	{
		const FVector A(Random.FRandRange(-8, 8), Random.FRandRange(-8, 8), Random.FRandRange(-5, 5));
		const FVector B = A + FVector(Random.FRandRange(-3, 3), Random.FRandRange(-3, 3), Random.FRandRange(-3, 3));
		const FVector Axis =
			FVector(Random.FRandRange(-1, 1), Random.FRandRange(-1, 1), Random.FRandRange(-1, 1)).GetSafeNormal();
		const double Angle = Random.FRandRange(.01, 2.79);
		auto F = PalmTests::Frame(A, B, FQuat(Axis, -Angle / 2), FQuat(Axis, Angle / 2));
		auto Q = PalmTests::Query(
			FVector(Random.FRandRange(-10, 10), Random.FRandRange(-10, 10), Random.FRandRange(-6, 6)),
			FVector(Random.FRandRange(-10, 10), Random.FRandRange(-10, 10), Random.FRandRange(-6, 6)), .5);
		Q.MotionStatistics = &Stats;
		FBallContact H;
		const double T0 = FPlatformTime::Seconds();
		const auto R = FBallHandCollisionSolver::Query(F, F.Colliders[0], Q, H);
		Times.Add((FPlatformTime::Seconds() - T0) * 1.e6);
		if (R == EBallQueryResult::Unresolved)
		{
			++Unresolved;
			continue;
		}
		if (R == EBallQueryResult::Contact)
			++Hits;
		else
			++Clear;
		// A dense oracle is a missed-penetration regression, not a mathematical CCD proof.
		for (int32 J = 0; J <= 2048; ++J)
		{
			const double T = double(J) / 2048;
			const double Sep =
				FBallHandCollisionSolver::Separation(F, F.Colliders[0], T, FMath::Lerp(Q.Start, Q.End, T), Q.Radius);
			if (Sep < -1.e-4)
			{
				if (R != EBallQueryResult::Contact || H.TimeOfImpact > T + 1.e-5)
				{
					AddError(FString::Printf(TEXT("Missed/late CCD seed case %d, sample %d, result %d, TOI %.12g"), I,
											 J, int32(R), H.TimeOfImpact));
					return false;
				}
				break;
			}
		}
		if (R == EBallQueryResult::Contact)
		{
			TestTrue(TEXT("Finite normalized geometric contact"),
					 H.Normal.IsUnit(1.e-8) && !H.Position.ContainsNaN() && !H.SurfaceVelocity.ContainsNaN());
			if (H.TimeOfImpact > 0)
				TestTrue(TEXT("Palm and ball contact points agree within tolerance"),
						 (H.Position - (FMath::Lerp(Q.Start, Q.End, H.TimeOfImpact) - H.Normal * Q.Radius)).Size() <
							 5.e-6);
		}
	}
	Times.Sort();
	AddInfo(FString::Printf(
		TEXT("PC_ROTATING_BOX_ORACLE Cases=1024 Seed=410120 DenseSamples=2049 Hits=%u Clear=%u Unresolved=%u "
			 "MedianUs=%.3f P95Us=%.3f P99Us=%.3f MaxUs=%.3f AverageIterations=%.3f MaxIterations=%u "
			 "DistanceEvaluations=%u; desktop only, dense oracle supplements analytic bounds"),
		Hits, Clear, Unresolved, Times[512], Times[972], Times[1013], Times.Last(), double(Stats.Iterations) / 1024,
		Stats.MaximumIterations, Stats.DistanceEvaluations));
	TestEqual(TEXT("Nominal seeded envelope converges within configured budget"), Unresolved, uint32(0));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmPairedPerformance,
								 "FourSquareVR.Ball.Palm.Performance.PairedEngineSimulationReplayAndMemory",
								 PalmPerformanceFlags)
bool FPalmPairedPerformance::RunTest(const FString &)
{
	TGuardValue<uint64> Restore(GFrameCounter, GFrameCounter);
	UWorld *World = UWorld::CreateWorld(EWorldType::Game, false);
	World->SpawnActor<ABallTestCourt>();
	FUnrealBallCollisionWorld Static;
	if (!Static.Initialize(World, 1))
	{
		World->DestroyWorld(false);
		return false;
	}
	FBallHandPoseHistory Raw;
	Raw.Initialize();
	for (bool Box : {false, true})
		for (double Speed : {1000.0, 6000.0, 12000.0})
		{
			auto S = BallTests::State();
			S.Position.Z = 500;
			S.LinearVelocity.X = Speed;
			auto C = PalmTests::Config();
			C.Radius = 3;
			FBallMotionHistory M;
			M.Initialize(256);
			FBallHistoryBuffer H;
			H.Initialize(256, S.Identity);
			FHistoricalBallCollisionWorld W(Static, M);
			FBallSimulator Sim;
			FBallStepOutput O;
			FBallState Reference;
			TArray<double> Times, ReplayTimes;
			Times.Reserve(512);
			ReplayTimes.Reserve(256);
			uint32 HandEvents = 0, WorldEvents = 0;
			uint64 Iterations = 0, Queries = 0;
			uint32 MaxIterations = 0;
			const SIZE_T MotionBytes = M.GetAllocatedBytes();
			for (int32 I = 0; I < 512; ++I)
			{
				++GFrameCounter;
				auto F = ProfilePalms(I, Box);
				auto Input = BallTests::Inputs(S);
				Input.MotionTimebase = Input.MotionRevision = 1;
				if (!M.Push(F) || H.Push({S, Input}) != EBallHistoryWrite::Stored ||
					Sim.SimulateStep(S, Input, C, W, O) != EBallStepResult::Success)
				{
					AddError(FString::Printf(TEXT("Paired profile Box=%d speed %.0f frame %d failed %d"), int32(Box),
											 Speed, I, int32(O.Result)));
					World->DestroyWorld(false);
					return false;
				}
				Times.Add(O.Diagnostics.DurationMicroseconds);
				Iterations += O.Diagnostics.MotionCCD.Iterations;
				Queries += O.Diagnostics.MotionCCD.Queries;
				MaxIterations = FMath::Max(MaxIterations, O.Diagnostics.MotionCCD.MaximumIterations);
				for (int32 J = 0; J < O.EventCount; ++J)
					if (O.Events[J].Category == EBallContactCategory::Hand)
						++HandEvents;
					else
						++WorldEvents;
				if (I == 509)
					Reference = S;
			}
			FBallReplayRunner Runner;
			FBallReplayReport Report;
			FBallState Replayed;
			for (int32 I = 0; I < 256; ++I)
			{
				if (Runner.Replay(H, S.Identity, 480, 510, C, {}, W, Replayed, Report) != EBallReplayResult::Success)
				{
					AddError(TEXT("Paired replay failed"));
					World->DestroyWorld(false);
					return false;
				}
				ReplayTimes.Add(Report.DurationMicroseconds);
			}
			TestTrue(TEXT("Full replay equivalence"), BallTests::ExactlyEqual(Reference, Replayed));
			TestTrue(TEXT("Actual hand/world collisions measured"), HandEvents > 0 && WorldEvents > 0);
			TestEqual(TEXT("No motion ring allocation growth"), M.GetAllocatedBytes(), MotionBytes);
			Times.Sort();
			ReplayTimes.Sort();
			AddInfo(FString::Printf(
				TEXT("PC_PAIRED Shape=%s Speed=%.0f StepMedianUs=%.3f StepP95Us=%.3f StepP99Us=%.3f StepMaxUs=%.3f "
					 "Replay30MedianUs=%.3f ReplayP95Us=%.3f ReplayP99Us=%.3f ReplayMaxUs=%.3f CCDQueries=%llu "
					 "CCDAverageIterations=%.3f CCDMaxIterations=%u HandEvents=%u WorldEvents=%u "
					 "MotionHistoryBytes=%llu BallHistoryBytes=%llu MotionFrameBytes=%llu SegmentBytes=%llu "
					 "ColliderBytes=%llu Raw2HandsBytes=%llu Output2Bytes=%llu ActorBytes=%llu StaticCacheBytes=%llu; "
					 "eight hands/four proxies, PC only, no XR/render/network/event ledger"),
				Box ? TEXT("RoundedBox") : TEXT("Sphere"), Speed, Times[256], Times[486], Times[506], Times.Last(),
				ReplayTimes[128], ReplayTimes[243], ReplayTimes[253], ReplayTimes.Last(), Queries,
				Queries ? double(Iterations) / Queries : 0, MaxIterations, HandEvents, WorldEvents, uint64(MotionBytes),
				uint64(H.GetAllocatedBytes()), uint64(sizeof(FBallMotionFrame)), uint64(sizeof(FBallMotionSegment)),
				uint64(sizeof(FBallColliderMotion)), uint64(2 * Raw.GetAllocatedBytes()),
				uint64(2 * sizeof(FBallStepOutput)), uint64(sizeof(AHandballBall)),
				uint64(Static.GetAllocatedBytes())));
		}
	World->DestroyWorld(false);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPalmMaximumWork, "FourSquareVR.Ball.Palm.Performance.MaximumKnotsWithFastRotation",
								 PalmPerformanceFlags)
bool FPalmMaximumWork::RunTest(const FString &)
{
	TGuardValue<uint64> Restore(GFrameCounter, GFrameCounter);
	UWorld *World = UWorld::CreateWorld(EWorldType::Game, false);
	World->SpawnActor<ABallTestCourt>();
	FUnrealBallCollisionWorld Static;
	if (!Static.Initialize(World, 1))
	{
		World->DestroyWorld(false);
		return false;
	}
	auto F = ProfilePalms(0, true, true);
	for (int32 I = 0; I < F.SegmentCount; ++I)
	{
		auto &S = F.Segments[I];
		S.StartOrientation =
			FQuat(FVector::UpVector, .65 * S.StartFraction) * FQuat(FVector::YAxisVector, UE_DOUBLE_PI / 2);
		S.EndOrientation =
			FQuat(FVector::UpVector, .65 * S.EndFraction) * FQuat(FVector::YAxisVector, UE_DOUBLE_PI / 2);
	}
	FBallMotionHistory H;
	H.Initialize(2);
	H.Push(F);
	FHistoricalBallCollisionWorld W(Static, H);
	FBallSimulator Sim;
	FBallStepOutput O;
	auto C = PalmTests::Config();
	C.Radius = 3;
	TArray<double> Times;
	Times.Reserve(256);
	uint32 Events = 0;
	for (int32 I = 0; I < 256; ++I)
	{
		++GFrameCounter;
		auto S = BallTests::State();
		S.Position = FVector(50, 0, 500);
		S.LinearVelocity.X = 1000;
		auto Input = BallTests::Inputs(S);
		Input.MotionTimebase = Input.MotionRevision = 1;
		if (Sim.SimulateStep(S, Input, C, W, O) != EBallStepResult::Success)
		{
			AddError(FString::Printf(TEXT("Fast-rotation maximum-knot fixture failed %d"), int32(O.Result)));
			World->DestroyWorld(false);
			return false;
		}
		Times.Add(O.Diagnostics.DurationMicroseconds);
		Events += O.EventCount;
	}
	TestTrue(TEXT("Capacity fixture actually collides"), Events > 0);
	TestTrue(TEXT("Maximum segment layout retained"), F.SegmentCount == 96);
	Times.Sort();
	AddInfo(FString::Printf(TEXT("PC_FAST_ROTATION_MAX_KNOTS Segments=96 AngularSpeed=78rad/s Slices=%u Queries=%u "
								 "CCDIterations=%u CCDMaxIterations=%u MedianUs=%.3f P95Us=%.3f P99Us=%.3f MaxUs=%.3f "
								 "Events=%u; capacity stress, no Quest qualification"),
							O.Diagnostics.MotionSlices, O.Diagnostics.Queries, O.Diagnostics.MotionCCD.Iterations,
							O.Diagnostics.MotionCCD.MaximumIterations, Times[128], Times[243], Times[253], Times.Last(),
							Events));
	World->DestroyWorld(false);
	return true;
}
#endif
