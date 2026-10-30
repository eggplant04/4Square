#include "Misc/AutomationTest.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "BallTestFixtures.h"
#include "Ball/BallTestCourt.h"
#include "Ball/BallSimulator.h"
#include "Ball/BallReplayRunner.h"
#include "Ball/BallHandPoseHistory.h"
#include "Ball/BallHandCollisionSolver.h"
#include "Ball/HistoricalBallCollisionWorld.h"
#include "Ball/UnrealBallCollisionWorld.h"
#include "Ball/VRHandCollisionComponent.h"
#include "Engine/World.h"
#include "HAL/PlatformTime.h"

namespace
{
constexpr auto Flags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;
struct FProfileWorld
{
	UWorld *W = UWorld::CreateWorld(EWorldType::Game, false);
	~FProfileWorld()
	{
		W->DestroyWorld(false);
	}
};
FBallMotionFrame MotionFrame(uint64 Frame, bool Stress = false)
{
	FBallMotionFrame F;
	F.Frame = Frame;
	F.Epoch = BallTests::State().Identity.Epoch;
	for (int32 K = 0; K < BallMotionLimits::Colliders; ++K)
	{
		FBallColliderMotion C;
		C.Id = K < 8 ? BallMotionLimits::HandId(K / 2 + 1, EBallHandSide(K % 2), 1)
					 : FBallColliderId{BallMotionLimits::KinematicNamespace | uint64(K - 7), 1};
		C.Participant = K < 8 ? K / 2 + 1 : 0;
		C.Side = EBallHandSide(K % 2);
		C.OriginRevision = 1;
		C.Category = K < 8 ? EBallContactCategory::Hand : EBallContactCategory::Kinematic;
		C.Material.bOverride = true;
		C.Material.Friction = 0;
		FBallMotionSegment Segments[8];
		const int32 N = Stress ? 8 : 1;
		for (int32 J = 0; J < N; ++J)
		{
			auto &S = Segments[J];
			S.StartFraction = J == 0 ? 0 : J / 8.0 + K * .0001;
			S.EndFraction = J == 7 ? 1 : (J + 1) / 8.0 + K * .0001;
			if (!Stress)
			{
				S.StartFraction = 0;
				S.EndFraction = 1;
			}
			const double A = (Frame + S.StartFraction) / 120.0, B = (Frame + S.EndFraction) / 120.0;
			S.Start = Stress ? FVector(1000 + J, 1000 + K * 20, 500)
							 : FVector(60 + 20 * FMath::Sin(2 * PI * A) + K * 30, K == 0 ? 0 : 100 + K * 20, 500);
			S.End = Stress ? FVector(1001 + J, 1000 + K * 20, 500)
						   : FVector(60 + 20 * FMath::Sin(2 * PI * B) + K * 30, K == 0 ? 0 : 100 + K * 20, 500);
			S.FirstSample = uint32(Frame + 1);
			S.SecondSample = uint32(Frame + 2);
		}
		check(F.Add(C, MakeArrayView(Segments, N)));
	}
	return F;
}
} // namespace
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandEnginePerformance,
								 "FourSquareVR.Ball.HandPerformance.EngineCourtEightHandsFourProxiesAndReplay", Flags)
bool FHandEnginePerformance::RunTest(const FString &)
{
	TGuardValue<uint64> Restore(GFrameCounter, GFrameCounter);
	FProfileWorld T;
	T.W->SpawnActor<ABallTestCourt>();
	FUnrealBallCollisionWorld Static;
	if (!Static.Initialize(T.W, 1))
	{
		AddError(TEXT("Engine query world failed"));
		return false;
	}
	FBallPhysicsConfig C;
	C.Gravity = FVector::ZeroVector;
	C.LinearDrag = C.AngularDrag = C.SurfaceFriction = C.SpinFriction = C.RollingResistance = 0;
	for (double Speed : {1000.0, 6000.0, 12000.0})
	{
		auto S = BallTests::State();
		S.Position.Z = 500;
		S.LinearVelocity.X = Speed;
		FBallMotionHistory Motions;
		Motions.Initialize(256);
		FBallHistoryBuffer H;
		H.Initialize(256, S.Identity);
		FHistoricalBallCollisionWorld W(Static, Motions);
		FBallSimulator Sim;
		FBallStepOutput O;
		FBallState Reference;
		TArray<double> Times;
		Times.Reserve(512);
		uint32 HandEvents = 0, WorldEvents = 0;
		for (int32 I = 0; I < 512; ++I)
		{
			++GFrameCounter;
			auto M = MotionFrame(I);
			auto Inputs = BallTests::Inputs(S);
			Inputs.MotionTimebase = Inputs.MotionRevision = 1;
			if (!Motions.Push(M) || H.Push({S, Inputs}) != EBallHistoryWrite::Stored ||
				Sim.SimulateStep(S, Inputs, C, W, O) != EBallStepResult::Success)
			{
				AddError(FString::Printf(TEXT("Combined engine scenario %.0f frame %d failed (%d)"), Speed, I,
										 int32(O.Result)));
				return false;
			}
			Times.Add(O.Diagnostics.DurationMicroseconds);
			for (int32 J = 0; J < O.EventCount; ++J)
			{
				if (O.Events[J].Category == EBallContactCategory::Hand)
				{
					++HandEvents;
				}
				else
				{
					++WorldEvents;
				}
			}
			if (I == 509)
			{
				Reference = S;
			}
		}
		TestTrue(TEXT("Benchmark exercises actual hand and world response"), HandEvents > 0 && WorldEvents > 0);
		FBallReplayRunner Runner;
		FBallReplayReport Report;
		FBallState Replayed;
		TArray<double> Replays;
		Replays.Reserve(256);
		const SIZE_T MotionBytes = Motions.GetAllocatedBytes(), StateBytes = H.GetAllocatedBytes();
		for (int32 I = 0; I < 256; ++I)
		{
			if (Runner.Replay(H, S.Identity, 480, 510, C, {}, W, Replayed, Report) != EBallReplayResult::Success)
			{
				AddError(TEXT("Combined historical replay failed"));
				return false;
			}
			Replays.Add(Report.DurationMicroseconds);
		}
		TestTrue(TEXT("Full replay state identical"), BallTests::ExactlyEqual(Reference, Replayed));
		TestEqual(TEXT("Motion memory remains bounded"), Motions.GetAllocatedBytes(), MotionBytes);
		TestEqual(TEXT("State memory remains bounded"), H.GetAllocatedBytes(), StateBytes);
		Times.Sort();
		Replays.Sort();
		AddInfo(FString::Printf(
			TEXT("PC_ENGINE_WORLD_HANDS Speed=%.0fcm/s Hands=8 Kinematics=4 StepMedianUs=%.3f StepP95Us=%.3f "
				 "StepP99Us=%.3f StepMaxUs=%.3f Replay30MedianUs=%.3f ReplayP95Us=%.3f ReplayP99Us=%.3f "
				 "ReplayMaxUs=%.3f HandEvents=%u WorldEvents=%u MotionHistoryBytes=%llu BallHistoryBytes=%llu "
				 "WorldCacheBytes=%llu; excludes rendering/XR sampling/network transport, NOT Quest qualification"),
			Speed, Times[256], Times[486], Times[506], Times.Last(), Replays[128], Replays[243], Replays[253],
			Replays.Last(), HandEvents, WorldEvents, uint64(MotionBytes), uint64(StateBytes),
			uint64(Static.GetAllocatedBytes())));
	}
	auto M = MotionFrame(0, true);
	FBallMotionHistory History;
	History.Initialize(2);
	History.Push(M);
	FHistoricalBallCollisionWorld StressWorld(Static, History);
	FBallSimulator Sim;
	FBallStepOutput O;
	TArray<double> Stress;
	Stress.Reserve(256);
	for (int32 I = 0; I < 256; ++I)
	{
		++GFrameCounter;
		auto S = BallTests::State();
		S.Position.Z = 500;
		S.LinearVelocity.X = 1000;
		auto Inputs = BallTests::Inputs(S);
		Inputs.MotionTimebase = Inputs.MotionRevision = 1;
		if (Sim.SimulateStep(S, Inputs, C, StressWorld, O) != EBallStepResult::Success)
		{
			AddError(TEXT("Maximum supported trajectory slicing failed"));
			return false;
		}
		Stress.Add(O.Diagnostics.DurationMicroseconds);
	}
	Stress.Sort();
	AddInfo(FString::Printf(
		TEXT("PC_MAXIMUM_MOTION_KNOTS Colliders=12 Segments=96 Slices=%u Queries=%u MedianUs=%.3f P95Us=%.3f "
			 "P99Us=%.3f MaxUs=%.3f; supported-capacity stress, NOT typical headset motion or Quest qualification"),
		O.Diagnostics.MotionSlices, O.Diagnostics.Queries, Stress[128], Stress[243], Stress[253], Stress.Last()));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandCapturePerformance,
								 "FourSquareVR.Ball.HandPerformance.BoundedTrajectoryReconstructionAndMemory", Flags)
bool FHandCapturePerformance::RunTest(const FString &)
{
	FBallHandPoseHistory Histories[8];
	FBallHandTrackingSettings T;
	FBallPalmSettings Palm;
	SIZE_T RawBytes = 0;
	for (int32 K = 0; K < 8; ++K)
	{
		Histories[K].Initialize();
		RawBytes += Histories[K].GetAllocatedBytes();
		for (int32 I = 0; I <= 240; ++I)
		{
			FBallHandObservation P;
			P.CaptureSeconds = P.TimelineSeconds = I / 120.0;
			P.Sequence = I + 1;
			P.ParticipantId = K / 2 + 1;
			P.Side = EBallHandSide(K % 2);
			P.ControllerPosition = FVector(K * 100 + I / 12.0, 0, 100);
			P.HeadPosition = P.ControllerPosition + FVector(0, 0, 50);
			Histories[K].Add(P, T);
		}
	}
	TArray<double> Times;
	Times.Reserve(1024);
	for (int32 I = 0; I < 1024; ++I)
	{
		const double Start = FPlatformTime::Seconds();
		FBallMotionFrame F;
		F.Epoch = BallTests::State().Identity.Epoch;
		F.Frame = 239;
		for (int32 K = 0; K < 8; ++K)
		{
			EBallTrackingRejection R;
			if (!Histories[K].BuildMotion(239 / 120.0, Palm, T, K / 2 + 1, EBallHandSide(K % 2), F, R))
			{
				AddError(TEXT("Snapshot reconstruction failed"));
				return false;
			}
		}
		Times.Add((FPlatformTime::Seconds() - Start) * 1.e6);
		TestEqual(TEXT("Eight numeric palm inputs"), F.ColliderCount, 8);
	}
	Times.Sort();
	AddInfo(FString::Printf(
		TEXT("PC_TRAJECTORY_BUILD Hands=8 RawSamplesPerHand=256 Iterations=1024 MedianUs=%.3f P95Us=%.3f P99Us=%.3f "
			 "MaxUs=%.3f RawHistory8HandsBytes=%llu RawHistoryLocal2HandsBytes=%llu MotionFrameBytes=%llu "
			 "BallStateBytes=%llu BallHistoryFrameBytes=%llu HandComponentBytes=%llu; fixed preallocations, excludes "
			 "engine/UObject allocator overhead and XR driver, NOT Quest qualification"),
		Times[512], Times[972], Times[1013], Times.Last(), uint64(RawBytes), uint64(RawBytes / 4),
		uint64(sizeof(FBallMotionFrame)), uint64(sizeof(FBallState)), uint64(sizeof(FBallHistoryFrame)),
		uint64(sizeof(UVRHandCollisionComponent))));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandKernelPerformance, "FourSquareVR.Ball.HandPerformance.RelativeSphereCCDKernel",
								 Flags)
bool FHandKernelPerformance::RunTest(const FString &)
{
	const auto F = MotionFrame(0);
	FBallCollisionQuery Q;
	Q.Start = FVector(0, 0, 500);
	Q.End = FVector(100, 0, 500);
	Q.Radius = 3;
	TArray<double> Times;
	Times.Reserve(256);
	uint32 Hits = 0;
	double Sum = 0;
	for (int32 Batch = 0; Batch < 256; ++Batch)
	{
		const double Start = FPlatformTime::Seconds();
		for (int32 I = 0; I < 2048; ++I)
		{
			FBallContact C;
			if (FBallHandCollisionSolver::Sweep(F, F.Colliders[I % 8], Q, C))
			{
				++Hits;
				Sum += C.TimeOfImpact;
			}
		}
		Times.Add((FPlatformTime::Seconds() - Start) * 1.e6);
	}
	FBallContact C;
	TestTrue(TEXT("Representative moving sphere contact"), FBallHandCollisionSolver::Sweep(F, F.Colliders[0], Q, C));
	const double Expected = 50.0 / (100 - (F.Segments[0].End.X - F.Segments[0].Start.X));
	TestEqual(TEXT("Kernel retains analytic TOI while profiled"), C.TimeOfImpact, Expected, 1.e-9);
	TestTrue(TEXT("Work retained"), Hits > 0 && Sum > 0);
	Times.Sort();
	AddInfo(FString::Printf(
		TEXT("PC_RELATIVE_SPHERE_CCD Sweeps=524288 BatchSweeps=2048 MedianBatchUs=%.3f P95BatchUs=%.3f P99BatchUs=%.3f "
			 "MedianUsPerSweep=%.6f Hits=%u; pure moving-sphere geometry only, NOT Quest qualification"),
		Times[128], Times[243], Times[253], Times[128] / 2048, Hits));
	return true;
}
#endif
