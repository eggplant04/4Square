#include "Misc/AutomationTest.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "BallTestFixtures.h"
#include "Ball/UnrealBallCollisionWorld.h"
#include "Ball/BallSimulationSubsystem.h"
#include "Ball/HandballBall.h"
#include "Ball/BallTestCourt.h"
#include "Ball/BallReplayRunner.h"
#include "Components/BoxComponent.h"
#include "Components/SphereComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/World.h"
#include "Engine/Engine.h"
#include "Engine/StaticMesh.h"
#include "PhysicsEngine/BodySetup.h"
#include "HAL/PlatformTime.h"

namespace
{
constexpr auto WorldFlags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;
struct FBallTestWorld
{
	UWorld *World = UWorld::CreateWorld(EWorldType::Game, false);
	~FBallTestWorld()
	{
		if (World)
		{
			World->DestroyWorld(false);
		}
	}
	UBoxComponent *Box(const TCHAR *Name, FVector Center, FVector Extent)
	{
		auto *Actor = World->SpawnActor<AActor>();
		auto *C = NewObject<UBoxComponent>(Actor, FName(Name));
		Actor->SetRootComponent(C);
		Actor->AddInstanceComponent(C);
		C->SetMobility(EComponentMobility::Static);
		C->SetBoxExtent(Extent);
		C->SetWorldLocation(Center);
		C->SetCollisionProfileName(TEXT("BlockAll"));
		C->SetSimulatePhysics(false);
		C->RegisterComponent();
		return C;
	}
};
FBallPhysicsConfig WorldConfig()
{
	FBallPhysicsConfig C;
	C.Gravity = FVector::ZeroVector;
	C.LinearDrag = C.AngularDrag = C.SurfaceFriction = C.SpinFriction = C.RollingResistance = 0;
	C.Restitution = 1;
	return C;
}
bool WorldStep(FBallState &S, const FBallPhysicsConfig &C, const IBallCollisionWorld &W, FBallStepOutput &O)
{
	FBallSimulator Sim;
	return Sim.SimulateStep(S, BallTests::Inputs(S), C, W, O) == EBallStepResult::Success;
}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallRealWorldTick, "FourSquareVR.Ball.World.ActualTickAndStartupBoundary", WorldFlags)
bool FBallRealWorldTick::RunTest(const FString &)
{
	TGuardValue<uint64> RestoreFrameCounter(GFrameCounter, GFrameCounter);
	FBallTestWorld T;
	GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(T.World);
	T.World->SpawnActor<ABallTestCourt>();
	auto *Ball = T.World->SpawnActor<AHandballBall>(FVector(0, 0, 100), FRotator::ZeroRotator);
	T.World->InitializeActorsForPlay(FURL());
	Ball->DispatchBeginPlay();
	auto *System = Ball->GetSimulationCoordinator();
	if (!TestNotNull(TEXT("Coordinator"), System))
	{
		GEngine->DestroyWorldContext(T.World);
		return false;
	}
	System->TickSimulation(.2); // Under the hard backlog limit, but larger than one catch-up budget.
	TestTrue(TEXT("Startup overrun declares a pause"), System->IsPaused());
	TestEqual(TEXT("No fabricated startup steps"), System->GetSimulationFrame(), uint64(0));
	for (int32 I = 0; I < 3; ++I)
	{
		System->TickSimulation(1.0 / 120);
	}
	TestFalse(TEXT("Stable startup establishes a new timebase"), System->IsPaused());
	TestEqual(TEXT("Startup recovery still has complete initial state"), System->GetSimulationFrame(), uint64(0));
	bool bBounced = false;
	for (int32 I = 0; I < 240; ++I)
	{
		++GFrameCounter;
		T.World->Tick(LEVELTICK_All, 1.0f / 120);
		bBounced |= Ball->GetBallLinearVelocity().Z > 100;
	}
	TestEqual(TEXT("Real registered tick advances 240 frames"), System->GetSimulationFrame(), uint64(240));
	TestTrue(TEXT("Real world gravity and bounce"), bBounced && Ball->GetBallPosition().Z >= 3);
	TestTrue(TEXT("Visual projection remains synchronized"),
			 Ball->GetActorLocation().Equals(Ball->GetBallPosition(), 1.e-8));
	TestEqual(TEXT("Real tick writes bounded history"), Ball->GetHistory().Num(), 240);
	GEngine->DestroyWorldContext(T.World);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallDropToRest, "FourSquareVR.Ball.World.FallingBallEventuallyRests", WorldFlags)
bool FBallDropToRest::RunTest(const FString &)
{
	FBallTestWorld T;
	T.World->SpawnActor<ABallTestCourt>();
	FUnrealBallCollisionWorld W;
	W.Initialize(T.World, 1);
	FBallPhysicsConfig Config;
	auto State = BallTests::State();
	State.Position.Z = 520;
	FBallSimulator Simulator;
	FBallStepOutput Output;
	for (int32 I = 0; I < 7200; ++I)
	{
		if (Simulator.SimulateStep(State, BallTests::Inputs(State), Config, W, Output) != EBallStepResult::Success)
		{
			AddError(FString::Printf(TEXT("Drop failed at %d, status %d"), I, static_cast<int32>(Output.Result)));
			return false;
		}
	}
	TestTrue(TEXT("Natural drop settles, not only an initially supported fixture"),
			 State.Activity == EBallActivityState::Resting);
	TestTrue(TEXT("Stable floor height"), FMath::IsNearlyEqual(State.Position.Z, 3.02, .021));
	AddInfo(FString::Printf(TEXT("Drop result Z=%.9f Vz=%.9f Support=%d"), State.Position.Z, State.LinearVelocity.Z,
							State.bHasSupport));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallEngineThinWall, "FourSquareVR.Ball.World.ThinWallAndNormals", WorldFlags)
bool FBallEngineThinWall::RunTest(const FString &)
{
	FBallTestWorld T;
	T.Box(TEXT("ThinWall"), FVector(.05, 0, 0), FVector(.05, 100, 100));
	FUnrealBallCollisionWorld W;
	TestTrue(TEXT("Register real static query geometry"), W.Initialize(T.World, 1));
	TestEqual(TEXT("One component"), W.NumComponents(), 1);
	FBallCollisionQuery Q;
	Q.Start = FVector(-50, 0, 0);
	Q.End = FVector(950, 0, 0);
	Q.Radius = 3;
	FBallContact Hit;
	TestTrue(TEXT("Unreal swept thin-wall contact"), W.SweepSphere(Q, Hit) == EBallQueryResult::Contact);
	TestTrue(TEXT("Engine TOI vs analytic within 20 micrometres"), FMath::IsNearlyEqual(Hit.TimeOfImpact, .047, 2.e-6));
	TestTrue(TEXT("Accurate obstacle-to-sphere normal"), Hit.Normal.Equals(-FVector::ForwardVector, 1.e-5));
	TestTrue(TEXT("Point on thin wall"), Hit.Position.Equals(FVector::ZeroVector, .002));
	auto C = WorldConfig();
	auto S = BallTests::State();
	S.Position = Q.Start;
	S.LinearVelocity = FVector(120000, 0, 0);
	FBallStepOutput O;
	TestTrue(TEXT("Real engine adapter drives custom response"), WorldStep(S, C, W, O));
	TestTrue(TEXT("Reflected velocity"), S.LinearVelocity.Equals(FVector(-120000, 0, 0), .01));
	TestTrue(TEXT("Remainder position within engine tolerance"), FMath::IsNearlyEqual(S.Position.X, -956.02, .005));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallEngineCorner, "FourSquareVR.Ball.World.MultiComponentCorner", WorldFlags)
bool FBallEngineCorner::RunTest(const FString &)
{
	FBallTestWorld T;
	T.Box(TEXT("Floor"), FVector(0, 0, -10), FVector(100, 100, 10));
	T.Box(TEXT("Wall"), FVector(-10, 0, 100), FVector(10, 100, 100));
	FUnrealBallCollisionWorld W;
	W.Initialize(T.World, 1);
	auto C = WorldConfig();
	auto S = BallTests::State();
	S.Position = FVector(10, 0, 10);
	S.LinearVelocity = FVector(-2400, 0, -2400);
	FBallStepOutput O;
	TestTrue(TEXT("Real separate-component manifold"), WorldStep(S, C, W, O));
	TestEqual(TEXT("Both corner impacts"), O.EventCount, 2);
	TestTrue(TEXT("Two normal responses"), S.LinearVelocity.Equals(FVector(2400, 0, 2400), .01));
	TestTrue(TEXT("Corner position"), S.Position.Equals(FVector(16.02, 0, 16.02), .005));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallEngineRest, "FourSquareVR.Ball.World.PenetrationAndExtendedRest", WorldFlags)
bool FBallEngineRest::RunTest(const FString &)
{
	FBallTestWorld T;
	T.Box(TEXT("Floor"), FVector(0, 0, -10), FVector(100, 100, 10));
	FUnrealBallCollisionWorld W;
	W.Initialize(T.World, 1);
	FBallPhysicsConfig C;
	auto S = BallTests::State();
	S.Position.Z = 1;
	S.LinearVelocity.Z = -100;
	FBallStepOutput O;
	TestTrue(TEXT("Engine MTD supports custom depenetration"), WorldStep(S, C, W, O));
	TestTrue(TEXT("Outside floor"), S.Position.Z >= 2.999 && S.Position.Z <= 3.041);
	FBallSimulator Sim;
	for (int32 I = 0; I < 3600; ++I)
	{
		if (Sim.SimulateStep(S, BallTests::Inputs(S), C, W, O) != EBallStepResult::Success)
		{
			AddError(TEXT("Engine resting trace failed"));
			return false;
		}
		if (S.Position.Z < 2.999 || S.Position.Z > 3.041)
		{
			AddError(TEXT("Engine resting position unstable"));
			return false;
		}
	}
	TestTrue(TEXT("Real geometry sleeping stable"),
			 S.Activity == EBallActivityState::Resting && S.LinearVelocity.IsZero());
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallEngineEpoch, "FourSquareVR.Ball.World.FiltersCapacityAndGeometryEpoch",
								 WorldFlags)
bool FBallEngineEpoch::RunTest(const FString &)
{
	FBallTestWorld T;
	auto *Box = T.Box(TEXT("Floor"), FVector(0, 0, -10), FVector(100, 100, 10));
	auto *Ignored = T.Box(TEXT("Ignored"), FVector(0, 0, -20), FVector(100, 100, 10));
	Ignored->ComponentTags.Add(TEXT("BallIgnore"));
	FUnrealBallCollisionWorld W;
	TestTrue(TEXT("Capture static epoch"), W.Initialize(T.World, 1));
	TestEqual(TEXT("Tag excludes collider"), W.NumComponents(), 1);
	FBallCollisionQuery Q;
	Q.Start = FVector(0, 0, 10);
	Q.End = FVector(0, 0, -10);
	Q.Radius = 3;
	Q.RequiredProfile = TEXT("InvisibleWall");
	FBallContact Hit;
	TestTrue(TEXT("Profile filter applies"), W.SweepSphere(Q, Hit) == EBallQueryResult::Clear);
	Q.RequiredProfile = NAME_None;
	TestTrue(TEXT("Default channel blocks"), W.SweepSphere(Q, Hit) == EBallQueryResult::Contact);
	Box->SetWorldLocation(FVector(0, 0, -11));
	TestEqual(TEXT("Moved geometry invalidates epoch"), W.GetRevision(), uint32(0));
	auto S = BallTests::State();
	S.Position.Z = 10;
	const auto Before = S;
	FBallSimulator Sim;
	FBallStepOutput O;
	auto C = WorldConfig();
	TestTrue(TEXT("Cannot replay against changed live world"),
			 Sim.SimulateStep(S, BallTests::Inputs(S), C, W, O) == EBallStepResult::IncompatibleEnvironment);
	TestTrue(TEXT("Complete state unchanged"), BallTests::ExactlyEqual(S, Before));
	TestTrue(TEXT("Explicit new epoch rebuild"), W.Initialize(T.World, 2));
	TestEqual(TEXT("New revision"), W.GetRevision(), uint32(2));
	T.Box(TEXT("NewRuntimeStatic"), FVector(0, 0, 0), FVector(1));
	TestEqual(TEXT("Spawned static detected after construction"), W.GetRevision(), uint32(0));
	Box->SetCollisionObjectType(ECC_WorldDynamic);
	TestTrue(TEXT("Static mobility with designer object type is supported"), W.Initialize(T.World, 3));
	Q.MaximumCandidates = 1;
	TestTrue(TEXT("Engine broadphase capacity is bounded"),
			 W.SweepSphere(Q, Hit) == EBallQueryResult::CapacityExceeded);
	Q.MaximumCandidates = 64;
	TestTrue(TEXT("Object type does not hide static geometry"), W.SweepSphere(Q, Hit) == EBallQueryResult::Contact);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallActorIntegration, "FourSquareVR.Ball.World.ActorLaunchResetAndSharedClock",
								 WorldFlags)
bool FBallActorIntegration::RunTest(const FString &)
{
	FBallTestWorld T;
	T.World->SpawnActor<ABallTestCourt>();
	auto *Ball = T.World->SpawnActor<AHandballBall>(FVector(0, 0, 100), FRotator::ZeroRotator);
	auto *System = T.World->GetSubsystem<UBallSimulationSubsystem>();
	if (!TestNotNull(TEXT("World subsystem exists"), System))
	{
		return false;
	}
	T.World->InitializeActorsForPlay(FURL());
	Ball->DispatchBeginPlay();
	TestTrue(TEXT("Ball registered with custom coordinator"), Ball->IsSimulationReady());
	TestFalse(TEXT("Collision proxy does not simulate Chaos"), Ball->CollisionProxy->IsSimulatingPhysics());
	TestFalse(TEXT("Visual mesh does not simulate Chaos"), Ball->VisualMesh->IsSimulatingPhysics());
	TestTrue(TEXT("Proxy is query-disabled"),
			 Ball->CollisionProxy->GetCollisionEnabled() == ECollisionEnabled::NoCollision);
	TestTrue(TEXT("Launch API"), Ball->LaunchBall(FVector(0, 0, -24000), FVector::ZeroVector));
	System->TickSimulation(1.0 / 120);
	TestEqual(TEXT("Exactly one fixed state"), Ball->GetSimulationState().SimulationFrame, uint64(1));
	TestTrue(TEXT("Real court bounce"), Ball->GetBallLinearVelocity().Z > 19000);
	TestTrue(TEXT("Actor projection matches sim output"),
			 Ball->GetActorLocation().Equals(Ball->GetBallPosition(), 1.e-8));
	TestEqual(TEXT("History stores complete pre-step boundary"), Ball->GetHistory().Num(), 1);
	const uint32 Generation = Ball->GetSimulationState().Identity.Generation;
	TestTrue(TEXT("Reset API"), Ball->ResetBall());
	TestTrue(TEXT("Reset to initial placement"), Ball->GetBallPosition().Equals(FVector(0, 0, 100), 1.e-8));
	TestTrue(TEXT("Reset changes generation, clears history"),
			 Ball->GetSimulationState().Identity.Generation == Generation + 1 && Ball->GetHistory().Num() == 0);
	auto Config = Ball->Physics;
	Config.Radius = 5;
	TestTrue(TEXT("Runtime settings at generation boundary"), Ball->SetBallPhysicsConfig(Config));
	TestTrue(TEXT("Proxy radius updated"),
			 FMath::IsNearlyEqual(static_cast<double>(Ball->CollisionProxy->GetUnscaledSphereRadius()), 5.0, 1.e-6));
	Config.FixedFrequency = 60;
	TestFalse(TEXT("No live per-ball rate change"), Ball->SetBallPhysicsConfig(Config));
	Ball->SetActorLocation(FVector(999, 0, 999));
	System->TickSimulation(1.0 / 120);
	TestTrue(TEXT("Actor transform never imported as physics state"),
			 Ball->GetBallPosition().X == 0 && Ball->GetActorLocation().Equals(Ball->GetBallPosition(), 1.e-8));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallEnginePerformance, "FourSquareVR.Ball.World.BoundedCollisionReplayPerformance",
								 WorldFlags)
bool FBallEnginePerformance::RunTest(const FString &)
{
	FBallTestWorld T;
	T.World->SpawnActor<ABallTestCourt>();
	FUnrealBallCollisionWorld W;
	if (!TestTrue(TEXT("Benchmark environment initialized"), W.Initialize(T.World, 1)))
	{
		return false;
	}
	const auto C = WorldConfig();
	for (double Speed : {1000.0, 3000.0, 10000.0, 32310.99})
	{
		auto S = BallTests::State();
		S.Position.Z = 500;
		S.LinearVelocity = FVector(24000, 12000, -18000).GetSafeNormal() * Speed;
		FBallHistoryBuffer H;
		H.Initialize(256, S.Identity);
		FBallSimulator Sim;
		FBallStepOutput O;
		FBallState Reference;
		TArray<double> Steps;
		Steps.Reserve(512);
		for (int32 I = 0; I < 512; ++I)
		{
			auto Inputs = BallTests::Inputs(S);
			H.Push({S, Inputs});
			if (Sim.SimulateStep(S, Inputs, C, W, O) != EBallStepResult::Success)
			{
				AddError(FString::Printf(TEXT("Engine benchmark failed at %.1fcm/s frame %d result %d"), Speed, I,
										 static_cast<int32>(O.Result)));
				return false;
			}
			Steps.Add(O.Diagnostics.DurationMicroseconds);
			if (I == 509)
			{
				Reference = S;
			}
		}
		const SIZE_T Bytes = W.GetAllocatedBytes();
		FBallReplayRunner Runner;
		FBallReplayReport Report;
		FBallState Replay;
		TArray<double> Times;
		Times.Reserve(256);
		const auto Before = W.GetStatistics();
		for (int32 I = 0; I < 256; ++I)
		{
			if (Runner.Replay(H, S.Identity, 480, 510, C, {}, W, Replay, Report) != EBallReplayResult::Success)
			{
				AddError(TEXT("Engine benchmark replay failed"));
				return false;
			}
			Times.Add(Report.DurationMicroseconds);
		}
		Times.Sort();
		Steps.Sort();
		TestTrue(TEXT("Real query replay reproduces all state"), BallTests::ExactlyEqual(Replay, Reference));
		TestEqual(TEXT("World cache storage bounded"), W.GetAllocatedBytes(), Bytes);
		FBallHistoryFrame Start;
		H.CopyFrame(480, S.Identity, Start);
		const auto &Stats = W.GetStatistics();
		AddInfo(FString::Printf(
			TEXT("PC_UNREAL_STATIC_COLLISION Iterations=256 FramesPerReplay=30 Speed=%.2fcm/s Components=%d "
				 "WorldCacheBytes=%llu HistoryBytes=%llu StepMedianUs=%.3f StepP95Us=%.3f MedianReplayUs=%.3f "
				 "P95ReplayUs=%.3f MaxReplayUs=%.3f ImpactsPerReplay=%u Queries=%llu SceneQueries=%llu "
				 "PeakCandidates=%u; excludes rendering/XR/networking, NOT Quest qualification"),
			Speed, W.NumComponents(), static_cast<uint64>(Bytes), static_cast<uint64>(H.GetAllocatedBytes()),
			Steps[256], Steps[486], Times[128], Times[243], Times.Last(),
			Reference.NextEventOrdinal - Start.State.NextEventOrdinal, Stats.Queries - Before.Queries,
			Stats.NarrowphaseQueries - Before.NarrowphaseQueries, Stats.PeakCandidates));
	}
	AddInfo(FString::Printf(TEXT("MEMORY NativeActorBytes=%llu TwoOutputBuffersBytes=%llu; excludes UObject "
								 "components/shared meshes/engine scene-query internals; history measured separately"),
							static_cast<uint64>(sizeof(AHandballBall)),
							static_cast<uint64>(2 * sizeof(FBallStepOutput))));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallCompoundCorner, "FourSquareVR.Ball.World.CompoundShapeCorner", WorldFlags)
bool FBallCompoundCorner::RunTest(const FString &)
{
	FBallTestWorld T;
	auto *Mesh = NewObject<UStaticMesh>(GetTransientPackage());
	Mesh->CreateBodySetup();
	auto *Setup = Mesh->GetBodySetup();
	Setup->CollisionTraceFlag = CTF_UseSimpleAsComplex;
	FKBoxElem Floor(200, 200, 20);
	Floor.Center = FVector(0, 0, -10);
	Setup->AggGeom.BoxElems.Add(Floor);
	FKBoxElem Wall(20, 200, 200);
	Wall.Center = FVector(-10, 0, 100);
	Setup->AggGeom.BoxElems.Add(Wall);
	Setup->CreatePhysicsMeshes();
	Mesh->SetExtendedBounds(FBoxSphereBounds(FBox(FVector(-20, -100, -20), FVector(100, 100, 200))));
	auto *Actor = T.World->SpawnActor<AActor>();
	auto *Component = NewObject<UStaticMeshComponent>(Actor);
	Actor->SetRootComponent(Component);
	Actor->AddInstanceComponent(Component);
	Component->SetMobility(EComponentMobility::Static);
	Component->SetStaticMesh(Mesh);
	Component->SetCollisionProfileName(TEXT("BlockAll"));
	Component->RegisterComponent();
	FUnrealBallCollisionWorld W;
	W.Initialize(T.World, 1);
	TestEqual(TEXT("One primitive with two shapes"), W.NumComponents(), 1);
	auto S = BallTests::State();
	S.Position = FVector(10, 0, 10);
	S.LinearVelocity = FVector(-2400, 0, -2400);
	FBallStepOutput O;
	TestTrue(TEXT("Compound simultaneous collision"), WorldStep(S, WorldConfig(), W, O));
	TestEqual(TEXT("Both subshape normals gathered"), O.ContactCount, 2);
	TestTrue(TEXT("Compound corner analytic response"),
			 S.LinearVelocity.Equals(FVector(2400, 0, 2400), .01) && S.Position.Equals(FVector(16.02, 0, 16.02), .005));
	return true;
}
#endif
