#include "Ball/BallNetworkTestRig.h"
#include "Ball/VRHandCollisionComponent.h"
#include "Ball/BallNetworkComponent.h"
#include "Ball/HandballBall.h"
#include "Ball/BallTestCourt.h"
#include "Ball/BallSimulationSubsystem.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "HAL/PlatformTime.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "MotionControllerComponent.h"

ABallNetworkTestPawn::ABallNetworkTestPawn()
{
	bReplicates = true;
	bAlwaysRelevant = true;
	SetReplicateMovement(false);
	Hands->bDebugDraw = false;
	bAnimateWrist = true;
	WristAmplitudeDegrees = 10;
	WristFrequencyHz = 1;
}
void ABallNetworkTestPawn::Tick(float Delta)
{
#if !UE_BUILD_SHIPPING
	if (FParse::Param(FCommandLine::Get(), TEXT("BallTestDynamicHands")))
	{
		Hands = FindComponentByClass<UVRHandCollisionComponent>();
		if (Hands)
		{
			Hands->bAllowScriptedTrackingForTests = true;
			Hands->bDebugDraw = false;
		}
	}
#endif
	if (IsLocallyControlled())
		Super::Tick(Delta);
}
void ABallNetworkTestPawn::BeginPlay()
{
	Super::BeginPlay();
#if !UE_BUILD_SHIPPING
	if (FParse::Param(FCommandLine::Get(), TEXT("BallTestDynamicHands")) && Hands)
	{
		Hands->DestroyComponent();
		Hands = nullptr;
		// The production subsystem discovers Grips and creates the replicated component on the server.
	}
#endif
}
ABallNetworkTestGameMode::ABallNetworkTestGameMode()
{
	PrimaryActorTick.bCanEverTick = true;
	DefaultPawnClass = ABallNetworkTestPawn::StaticClass();
	PlayerControllerClass = ABallNetworkTestController::StaticClass();
}
void ABallNetworkTestGameMode::Tick(float Delta)
{
	Super::Tick(Delta);
#if !UE_BUILD_SHIPPING
	if (!FParse::Param(FCommandLine::Get(), TEXT("BallNetworkIPTest")))
		return;
	const double Now = FPlatformTime::Seconds();
	if (!Started)
		Started = Now;
	double Duration = 55;
	FParse::Value(FCommandLine::Get(), TEXT("BallNetTestSeconds="), Duration);
	if (Now - Started > Duration)
		FPlatformMisc::RequestExit(false);
#endif
}
void ABallNetworkTestGameMode::BeginPlay()
{
	Super::BeginPlay();
#if !UE_BUILD_SHIPPING
	if (!FParse::Param(FCommandLine::Get(), TEXT("BallNetworkIPTest")))
		return;
	GetWorld()->SpawnActor<ABallTestCourt>();
	FTransform T(FVector(0, 0, 100));
	auto *Ball = GetWorld()->SpawnActorDeferred<AHandballBall>(AHandballBall::StaticClass(), T);
	Ball->Physics.Gravity = FVector::ZeroVector;
	Ball->FinishSpawning(T);
#endif
}
void ABallNetworkTestController::Tick(float Delta)
{
	Super::Tick(Delta);
#if !UE_BUILD_SHIPPING
	if (!FParse::Param(FCommandLine::Get(), TEXT("BallNetworkIPTest")))
		return;
	const double Now = FPlatformTime::Seconds();
	if (!Started)
		Started = Now;
	if (Now >= NextReport)
	{
		for (TActorIterator<AHandballBall> It(GetWorld()); It; ++It)
		{
			UE_LOG(LogTemp, Display, TEXT("NETTEST Role=%s %s"), HasAuthority() ? TEXT("Server") : TEXT("Client"),
				   *It->Network->GetNetworkDiagnostics());
			if (!HasAuthority() && !bClientGuardTest && It->IsSimulationReady())
			{
				bClientGuardTest = true;
				const bool Modified =
					It->SetBallState(FVector(9999), FVector(9999), FRotator::ZeroRotator, FVector::ZeroVector);
				UE_LOG(LogTemp, Display, TEXT("NETTEST ClientAuthorityGuard=%s"),
					   Modified ? TEXT("FAIL") : TEXT("PASS"));
			}
			if (HasAuthority() && !bReset && Now - Started > 15)
			{
				bReset = true;
				It->SetBallState(FVector(0, 0, 100), FVector::ZeroVector, FRotator::ZeroRotator, FVector::ZeroVector);
				UE_LOG(LogTemp, Display, TEXT("NETTEST ResetGeneration=%u"),
					   It->GetSimulationState().Identity.Generation);
			}
		}
		for (TActorIterator<APawn> It(GetWorld()); It; ++It)
			if (auto *H = It->FindComponentByClass<UVRHandCollisionComponent>())
			{
				UE_LOG(LogTemp, Display, TEXT("NETTEST Hands Local=%d %s"), It->IsLocallyControlled(),
					   *H->GetNetworkDiagnostics());
				if (!HasAuthority() && !It->IsLocallyControlled() && !bOwnershipProbe &&
					H->GetAuthorityProfile().IsValid())
				{
					bOwnershipProbe = true;
					struct FParams
					{
						FBallWirePacket Packet;
					} Parameters;
					// Deliberately attempt the real RPC on an unowned Pawn; UE must deny this actor channel.
					if (UFunction *Function = H->FindFunction(TEXT("ServerSubmitPoses")))
						H->ProcessEvent(Function, &Parameters);
					UE_LOG(LogTemp, Display, TEXT("NETTEST AttemptedUnownedPoseRPC Participant=%u"),
						   H->GetParticipantId());
				}
				if (HasAuthority() && !bConfiguration && Now - Started > 25)
				{
					bConfiguration = true;
					auto L = H->LeftPalm;
					L.RoundRadius = .7;
					UE_LOG(LogTemp, Display, TEXT("NETTEST ConfigurationBoundary=%s"),
						   H->SetServerPalmConfiguration(L, H->RightPalm) ? TEXT("PASS") : TEXT("FAIL"));
				}
			}
		if (auto *S = GetWorld()->GetSubsystem<UBallSimulationSubsystem>())
			UE_LOG(LogTemp, Display, TEXT("NETTEST Simulation %s"), *S->GetSimulationDiagnostics());
		NextReport = Now + 2;
	}
	double Duration = 45;
	FParse::Value(FCommandLine::Get(), TEXT("BallNetTestSeconds="), Duration);
	if (Now - Started > Duration)
		FPlatformMisc::RequestExit(false);
#endif
}
