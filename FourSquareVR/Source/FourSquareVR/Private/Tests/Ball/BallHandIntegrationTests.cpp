#include "Misc/AutomationTest.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "BallTestFixtures.h"
#include "Ball/BallHandTestRig.h"
#include "Ball/VRHandCollisionComponent.h"
#include "Ball/BallKinematicColliderComponent.h"
#include "Ball/BallSimulationSubsystem.h"
#include "Ball/HandballBall.h"
#include "Ball/BallReplayRunner.h"
#include "Ball/HistoricalBallCollisionWorld.h"
#include "Ball/UnrealBallCollisionWorld.h"
#include "MotionControllerComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/BoxComponent.h"
#include "Engine/Engine.h"
#include "Engine/World.h"

namespace
{
constexpr auto Flags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;
struct FHandWorld
{
	TGuardValue<uint64> RestoreCounter{GFrameCounter, GFrameCounter};
	UWorld *World = UWorld::CreateWorld(EWorldType::Game, false);
	FHandWorld()
	{
		GEngine->CreateNewWorldContext(EWorldType::Game).SetCurrentWorld(World);
	}
	~FHandWorld()
	{
		GEngine->DestroyWorldContext(World);
		World->DestroyWorld(false);
	}
	void Start(TConstArrayView<AActor *> Actors)
	{
		World->InitializeActorsForPlay(FURL());
		for (auto *A : Actors)
		{
			A->DispatchBeginPlay();
		}
	}
	void Tick(int32 Count = 1)
	{
		for (int32 I = 0; I < Count; ++I)
		{
			++GFrameCounter;
			World->Tick(LEVELTICK_All, 1.0f / 120);
		}
	}
	AHandballBall *Ball(FVector Position = FVector(0, 0, 100))
	{
		auto *B = World->SpawnActor<AHandballBall>(Position, FRotator::ZeroRotator);
		B->Physics.Gravity = FVector::ZeroVector;
		B->Physics.LinearDrag = B->Physics.AngularDrag = 0;
		B->bAutoRegisterVRHands = false;
		return B;
	}
};
} // namespace
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandNativeBinding,
								 "FourSquareVR.Ball.HandIntegration.ExistingGripRegistrationAndDiscovery", Flags)
bool FHandNativeBinding::RunTest(const FString &)
{
	FHandWorld T;
	auto *Rig = T.World->SpawnActor<ABallHandTestRig>();
	Rig->Hands->bDebugDraw = false;
	TestTrue(TEXT("Existing Grip sources discovered"), Rig->Hands->DiscoverGripControllers());
	TestTrue(TEXT("Exact existing references preserved"),
			 Rig->Hands->LeftGrip == Rig->Left && Rig->Hands->RightGrip == Rig->Right);
	TestFalse(TEXT("Swapped controllers rejected"),
			  Rig->Hands->RegisterMotionControllers(Rig->Right, Rig->Left, Rig->Head, Rig->TrackingOrigin));
	auto *Aim = NewObject<UMotionControllerComponent>(Rig);
	Aim->SetupAttachment(Rig->TrackingOrigin);
	Aim->SetTrackingMotionSource(TEXT("LeftAim"));
	Rig->AddInstanceComponent(Aim);
	Aim->RegisterComponent();
	UMotionControllerComponent *Left = nullptr, *Right = nullptr;
	TestTrue(TEXT("Aim controller ignored"), UVRHandCollisionComponent::FindUniqueGrips(Rig, Left, Right));
	auto *Duplicate = NewObject<UMotionControllerComponent>(Rig);
	Duplicate->SetupAttachment(Rig->TrackingOrigin);
	Duplicate->SetTrackingMotionSource(TEXT("LeftGrip"));
	Rig->AddInstanceComponent(Duplicate);
	Duplicate->RegisterComponent();
	TestFalse(TEXT("Ambiguous duplicate sources not guessed"),
			  UVRHandCollisionComponent::FindUniqueGrips(Rig, Left, Right));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandNativeStrike, "FourSquareVR.Ball.HandIntegration.RealTicksCaptureAndStrike",
								 Flags)
bool FHandNativeStrike::RunTest(const FString &)
{
	FHandWorld T;
	auto *Rig = T.World->SpawnActor<ABallHandTestRig>();
	Rig->Hands->bDebugDraw = false;
	auto *Ball = T.Ball();
	AActor *Actors[] = {Rig, Ball};
	T.Start(MakeArrayView(Actors));
	bool Strike = false;
	double MaxSpeed = 0;
	for (int32 I = 0; I < 360; ++I)
	{
		T.Tick();
		const auto &O = Ball->GetLastStepOutput();
		for (int32 J = 0; J < O.EventCount; ++J)
		{
			Strike |= O.Events[J].Category == EBallContactCategory::Hand;
		}
		MaxSpeed = FMath::Max(MaxSpeed, Ball->GetBallLinearVelocity().Size());
	}
	TestTrue(TEXT("Actual tracked-component recording pipeline strikes stationary ball"), Strike && MaxSpeed > 50);
	TestTrue(TEXT("Native references remain bound"), Rig->Hands->AreControllersRegistered());
	TestTrue(TEXT("Ready timestamped history"),
			 Rig->Hands->GetHandHistory(EBallHandSide::Left).GetLatest()->bCollisionReady);
	TestTrue(TEXT("Visual state projection"), Ball->GetActorLocation().Equals(Ball->GetBallPosition(), 1.e-5));
	auto *System = Ball->GetSimulationCoordinator();
	TestFalse(TEXT("No runaway/pause"), System->IsPaused());
	const auto *Frame =
		System->GetMotionHistory().Find(System->GetSimulationFrame() - 1, System->GetTrackingTimebase(), 1);
	TestTrue(TEXT("Both palm trajectories recorded"), Frame && Frame->ColliderCount == 2);
	AddInfo(System->GetSimulationDiagnostics());
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandNativeDiscontinuities,
								 "FourSquareVR.Ball.HandIntegration.TeleportLossAndReacquisition", Flags)
bool FHandNativeDiscontinuities::RunTest(const FString &)
{
	FHandWorld T;
	auto *Rig = T.World->SpawnActor<ABallHandTestRig>();
	Rig->Hands->bDebugDraw = false;
	Rig->bAnimate = false;
	auto *Ball = T.Ball(FVector(500, 0, 100));
	AActor *Actors[] = {Rig, Ball};
	T.Start(MakeArrayView(Actors));
	T.Tick(4);
	const uint32 OldGeneration = Rig->Hands->GetHandHistory(EBallHandSide::Left).GetLatest()->Generation;
	Rig->SetActorLocation(FVector(10000, 0, 0));
	T.Tick();
	const auto *New = Rig->Hands->GetHandHistory(EBallHandSide::Left).GetLatest();
	TestTrue(TEXT("Origin jump invalidates continuity"), New->Generation > OldGeneration && !New->bCollisionReady);
	TestTrue(TEXT("Teleport creates no ball impulse"), Ball->GetBallLinearVelocity().IsZero());
	T.Tick(2);
	TestTrue(TEXT("Fresh coherent tracking re-enables"),
			 Rig->Hands->GetHandHistory(EBallHandSide::Left).GetLatest()->bCollisionReady);
	Rig->Hands->bEnableLeft = false;
	T.Tick();
	TestTrue(TEXT("Loss disables collision data"),
			 !Rig->Hands->GetHandHistory(EBallHandSide::Left).GetLatest()->bCollisionReady);
	Rig->Hands->bEnableLeft = true;
	T.Tick();
	TestTrue(TEXT("Reacquisition first sample remains safe"),
			 !Rig->Hands->GetHandHistory(EBallHandSide::Left).GetLatest()->bCollisionReady);
	T.Tick();
	TestTrue(TEXT("Reacquired continuity"),
			 Rig->Hands->GetHandHistory(EBallHandSide::Left).GetLatest()->bCollisionReady);
	const uint32 OldParticipant = Rig->Hands->GetParticipantId();
	Rig->Hands->bAllowScriptedTrackingForTests = false;
	T.Tick();
	TestEqual(TEXT("Unpossessed/nonlocal source releases capture ownership"), Rig->Hands->GetParticipantId(),
			  uint32(0));
	Rig->Hands->bAllowScriptedTrackingForTests = true;
	T.Tick(2);
	TestTrue(TEXT("New local lifetime can reacquire capture"), Rig->Hands->GetParticipantId() > OldParticipant);
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandNativeWorldOrigin,
								 "FourSquareVR.Ball.HandIntegration.WorldOriginCreatesExplicitHistoryBoundary", Flags)
bool FHandNativeWorldOrigin::RunTest(const FString &)
{
	FHandWorld T;
	auto *Ball = T.Ball(FVector(10, 20, 100));
	AActor *Actors[] = {Ball};
	T.Start(MakeArrayView(Actors));
	T.Tick(3);
	auto *System = Ball->GetSimulationCoordinator();
	const auto Before = Ball->GetSimulationState();
	const uint32 Timebase = System->GetTrackingTimebase();
	TestTrue(TEXT("Actual engine origin rebase"), T.World->SetNewWorldOrigin(FIntVector(10000, 0, 0)));
	T.Tick();
	TestTrue(TEXT("New generation and timebase"),
			 Ball->GetSimulationState().Identity.Generation == Before.Identity.Generation + 1 &&
				 System->GetTrackingTimebase() == Timebase + 1);
	TestTrue(TEXT("Translated simulation position exactly once"),
			 Ball->GetBallPosition().Equals(Before.Position - FVector(10000, 0, 0), 1.e-5));
	TestTrue(TEXT("Projected visual matches"), Ball->GetActorLocation().Equals(Ball->GetBallPosition(), 1.e-5));
	TestFalse(TEXT("Rebase does not leave world paused"), System->IsPaused());
	TestTrue(TEXT("Old environment inputs unavailable"), !System->GetMotionHistory().Find(0, Timebase, 1));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandNativeKinematic,
								 "FourSquareVR.Ball.HandIntegration.RecordedBoxSurvivesSourceDestruction", Flags)
bool FHandNativeKinematic::RunTest(const FString &)
{
	FHandWorld T;
	auto *Ball = T.Ball(FVector::ZeroVector);
	auto *Obstacle = T.World->SpawnActor<AActor>();
	auto *Box = NewObject<UBoxComponent>(Obstacle);
	Obstacle->SetRootComponent(Box);
	Obstacle->AddInstanceComponent(Box);
	Box->SetMobility(EComponentMobility::Movable);
	Box->SetBoxExtent(FVector(5));
	Box->SetWorldLocation(FVector(-20, 0, 0));
	Box->SetCollisionProfileName(TEXT("BlockAll"));
	Box->RegisterComponent();
	auto *Recorder = NewObject<UBallKinematicColliderComponent>(Obstacle);
	Obstacle->AddInstanceComponent(Recorder);
	Recorder->TrackedComponent = Box;
	Recorder->Shape = EBallKinematicShape::TranslatingBox;
	Recorder->BoxHalfExtents = FVector(5);
	Recorder->RegisterComponent();
	AActor *Actors[] = {Obstacle, Ball};
	T.Start(MakeArrayView(Actors));
	T.Tick(3);
	Obstacle->SetActorLocation(FVector(-10, 0, 0));
	T.Tick();
	Obstacle->SetActorLocation(FVector::ZeroVector);
	T.Tick();
	TestTrue(TEXT("Moving historical box supplies collision impulse"), Ball->GetBallLinearVelocity().X > 1000);
	T.Tick(7);
	const auto Expected = Ball->GetSimulationState();
	auto *System = Ball->GetSimulationCoordinator();
	Obstacle->SetActorRotation(FRotator(0, 45, 0));
	T.Tick();
	TestTrue(TEXT("Unsupported rotating box explicitly disabled"),
			 Recorder->Rejection == EBallTrackingRejection::UnsupportedRotation);
	Obstacle->Destroy();
	FUnrealBallCollisionWorld Static;
	TestTrue(TEXT("Static adapter remains usable without live source"),
			 Static.Initialize(T.World, Expected.EnvironmentRevision));
	FHistoricalBallCollisionWorld W(Static, System->GetMotionHistory());
	FBallReplayRunner Runner;
	FBallReplayReport Report;
	FBallState Replayed;
	TestTrue(TEXT("Replay does not read destroyed component"),
			 Runner.Replay(Ball->GetHistory(), Expected.Identity, 0, Expected.SimulationFrame, Ball->Physics,
						   Ball->HistorySettings, W, Replayed, Report) == EBallReplayResult::Success);
	TestTrue(TEXT("Complete numerical state reproduced"), BallTests::ExactlyEqual(Expected, Replayed));
	return true;
}
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FHandNativeTuning,
								 "FourSquareVR.Ball.HandIntegration.LivePalmTuningRequiresFreshSamples", Flags)
bool FHandNativeTuning::RunTest(const FString &)
{
	FHandWorld T;
	auto *Rig = T.World->SpawnActor<ABallHandTestRig>();
	Rig->Hands->bDebugDraw = false;
	Rig->bAnimate = false;
	auto *Ball = T.Ball(FVector(500, 0, 100));
	AActor *Actors[] = {Rig, Ball};
	T.Start(MakeArrayView(Actors));
	T.Tick(4);
	auto *System = Ball->GetSimulationCoordinator();
	FBallMotionFrame F;
	F.Epoch = Ball->GetSimulationState().Identity.Epoch;
	F.Frame = System->GetSimulationFrame();
	F.Timebase = System->GetTrackingTimebase();
	Rig->Hands->LeftPalm.Radius = 20;
	Rig->Hands->BuildRecordedMotion(F, F.Frame / 120.0);
	TestEqual(TEXT("Late-frame Blueprint tuning cannot reinterpret old trajectories"), F.ColliderCount, 0);
	T.Tick(3);
	const auto *Recorded =
		System->GetMotionHistory().Find(System->GetSimulationFrame() - 1, System->GetTrackingTimebase(), 1);
	bool Found = false;
	if (Recorded)
	{
		for (int32 I = 0; I < Recorded->ColliderCount; ++I)
		{
			const auto &C = Recorded->Colliders[I];
			Found |= C.Side == EBallHandSide::Left && C.Radius == 20 && C.ConfigurationVersion == 2;
		}
	}
	TestTrue(TEXT("New profile recorded after valid continuity"), Found);
	TestTrue(TEXT("No tuning impulse"), Ball->GetBallLinearVelocity().IsZero());
	return true;
}
#endif
