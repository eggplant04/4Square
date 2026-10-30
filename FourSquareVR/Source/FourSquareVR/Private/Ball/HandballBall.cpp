#include "Ball/HandballBall.h"
#include "Ball/BallSimulationSubsystem.h"
#include "Components/SceneComponent.h"
#include "Components/SphereComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "DrawDebugHelpers.h"
#include "HAL/IConsoleManager.h"
#include "UObject/ConstructorHelpers.h"

static TAutoConsoleVariable<int32> CVarBallDebug(TEXT("ball.DebugDraw"), 0,
												 TEXT("Draw custom ball collision/state/contacts."));

AHandballBall::AHandballBall()
	: PreparedOutput(MakeUnique<FBallStepOutput>()), LastOutputStorage(MakeUnique<FBallStepOutput>())
{
	PrimaryActorTick.bCanEverTick = false;
	bReplicates = false; // Stage 5 will enable replication with the complete authoritative state protocol.
	SetReplicateMovement(false);
	SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("SimulationRoot"));
	SetRootComponent(SceneRoot);
	CollisionProxy = CreateDefaultSubobject<USphereComponent>(TEXT("BallCollisionProxy"));
	CollisionProxy->SetupAttachment(SceneRoot);
	CollisionProxy->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	CollisionProxy->SetSimulatePhysics(false);
	CollisionProxy->SetGenerateOverlapEvents(false);
	VisualMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("VisualMesh"));
	VisualMesh->SetupAttachment(SceneRoot);
	VisualMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
	VisualMesh->SetSimulatePhysics(false);
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Sphere(TEXT("/Engine/BasicShapes/Sphere.Sphere"));
	if (Sphere.Succeeded())
	{
		VisualMesh->SetStaticMesh(Sphere.Object);
	}
}
AHandballBall::~AHandballBall() = default;

void AHandballBall::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	SceneRoot->SetWorldScale3D(FVector::OneVector); // Physical radius is explicit configuration, not Actor scale.
	if (!bInitialized && Physics.IsValid())
	{
		CollisionProxy->SetSphereRadius(Physics.Radius, false);
		VisualMesh->SetRelativeScale3D(FVector(Physics.Radius / 50.0));
	}
}

void AHandballBall::BeginPlay()
{
	Super::BeginPlay();
	InitialTransform = GetActorTransform();
	if (GetNetMode() == NM_Client)
	{
		return;
	} // Custom snapshot/prediction transport is Stage 5/6.
	if (auto* System = GetWorld()->GetSubsystem<UBallSimulationSubsystem>())
	{
		if (!System->RegisterBall(this))
		{
			UE_LOG(LogHandballBall, Error, TEXT("Ball registration failed: check settings/frequency/capacity."));
		}
	}
}

void AHandballBall::EndPlay(const EEndPlayReason::Type Reason)
{
	if (auto* System = GetWorld()->GetSubsystem<UBallSimulationSubsystem>())
	{
		System->UnregisterBall(this);
	}
	bInitialized = false;
	Super::EndPlay(Reason);
}

bool AHandballBall::InitializeSimulation(FBallIdentity Identity, uint64 Frame, uint32 Environment)
{
	if (!Physics.IsValid() || !HistorySettings.IsValid(Physics.FixedFrequency) || Environment == 0)
	{
		return false;
	}
	RuntimePhysics = Physics;
	FBallState Candidate;
	Candidate.Identity = Identity;
	Candidate.SimulationFrame = Frame;
	Candidate.EnvironmentRevision = Environment;
	Candidate.ConfigVersion = RuntimePhysics.Version;
	Candidate.Position = bInitialized ? State.Position : InitialTransform.GetLocation();
	Candidate.Rotation = bInitialized ? State.Rotation : InitialTransform.GetRotation();
	Candidate.LinearVelocity = bInitialized ? State.LinearVelocity : InitialLinearVelocity;
	Candidate.AngularVelocity = bInitialized ? State.AngularVelocity : InitialAngularVelocity;
	if (!Candidate.IsValid() || !History.Initialize(HistorySettings.HistoryCapacity, Identity))
	{
		return false;
	}
	State = Candidate;
	bInitialized = true;
	CollisionProxy->SetSphereRadius(RuntimePhysics.Radius, false);
	VisualMesh->SetRelativeScale3D(FVector(RuntimePhysics.Radius / 50.0));
	ProjectState();
	return true;
}

bool AHandballBall::PrepareStep(const IBallCollisionWorld& World, uint64 Frame)
{
	if (!bInitialized || State.SimulationFrame != Frame)
	{
		return false;
	}
	PreparedInputs = {};
	PreparedInputs.SimulationFrame = Frame;
	PreparedInputs.ConfigVersion = State.ConfigVersion;
	PreparedInputs.MaterialVersion = State.MaterialVersion;
	PreparedInputs.EnvironmentRevision = State.EnvironmentRevision;
	if (History.CanPush({State, PreparedInputs}) != EBallHistoryWrite::Stored)
	{
		return false;
	}
	PreparedState = State;
	const auto Result = Simulator.SimulateStep(PreparedState, PreparedInputs, RuntimePhysics, World, *PreparedOutput);
	if (Result != EBallStepResult::Success)
	{
		UE_LOG(LogHandballBall, Error, TEXT("Ball step failed (%d), frame %llu; state remains unchanged."),
			   static_cast<int32>(Result), Frame);
		return false;
	}
	return true;
}

void AHandballBall::CommitStep()
{
	check(History.Push({State, PreparedInputs}) == EBallHistoryWrite::Stored);
	State = PreparedState;
	*LastOutputStorage = *PreparedOutput;
	ProjectState();
}
void AHandballBall::ProjectState()
{
	// Transform is an OUTPUT. Actor movement/overlap callbacks never feed the solver.
	SceneRoot->SetWorldScale3D(FVector::OneVector);
	SceneRoot->SetWorldLocationAndRotation(State.Position, State.Rotation, false, nullptr,
										   ETeleportType::TeleportPhysics);
}

bool AHandballBall::SetBallState(FVector Position, FVector Velocity, FRotator Rotation, FVector AngularVelocity)
{
	if (!HasAuthority() || !bInitialized || State.Identity.Generation == MAX_uint32)
	{
		return false;
	}
	FBallState Candidate;
	Candidate.Identity = State.Identity;
	++Candidate.Identity.Generation;
	Candidate.SimulationFrame = State.SimulationFrame;
	Candidate.ConfigVersion = State.ConfigVersion;
	Candidate.MaterialVersion = State.MaterialVersion;
	Candidate.EnvironmentRevision = State.EnvironmentRevision;
	Candidate.Position = Position;
	Candidate.LinearVelocity = Velocity;
	Candidate.Rotation = Rotation.Quaternion();
	Candidate.AngularVelocity = AngularVelocity;
	if (!Candidate.IsValid())
	{
		return false;
	}
	State = Candidate;
	History.Reset(State.Identity);
	*LastOutputStorage = {};
	ProjectState();
	return true;
}
bool AHandballBall::ResetBall()
{
	return SetBallState(InitialTransform.GetLocation(), FVector::ZeroVector, InitialTransform.Rotator(),
						FVector::ZeroVector);
}
bool AHandballBall::LaunchConfiguredBall()
{
	return LaunchBall(InitialLinearVelocity, InitialAngularVelocity);
}
bool AHandballBall::LaunchBall(FVector Velocity, FVector AngularVelocity)
{
	return SetBallState(State.Position, Velocity, State.Rotation.Rotator(), AngularVelocity);
}

bool AHandballBall::SetBallPhysicsConfig(const FBallPhysicsConfig& Config)
{
	if (!HasAuthority() || !bInitialized || !Config.IsValid() ||
		Config.FixedFrequency != RuntimePhysics.FixedFrequency || RuntimePhysics.Version == MAX_int32 ||
		State.Identity.Generation == MAX_uint32)
	{
		return false;
	}
	Physics = Config;
	Physics.Version = RuntimePhysics.Version + 1;
	RuntimePhysics = Physics;
	State.ConfigVersion = Physics.Version;
	CollisionProxy->SetSphereRadius(Physics.Radius, false);
	VisualMesh->SetRelativeScale3D(FVector(Physics.Radius / 50.0));
	return SetBallState(State.Position, State.LinearVelocity, State.Rotation.Rotator(), State.AngularVelocity);
}

void AHandballBall::DrawDiagnostics() const
{
	if (!bDebugDraw && CVarBallDebug.GetValueOnGameThread() == 0)
	{
		return;
	}
	UWorld* World = GetWorld();
	const FBallStepOutput& LastOutput = *LastOutputStorage;
	DrawDebugSphere(World, State.Position, RuntimePhysics.Radius, 16, IsBallResting() ? FColor::Green : FColor::Cyan);
	DrawDebugLine(World, State.Position, State.Position + State.LinearVelocity * 0.05, FColor::Yellow);
	for (int32 I = 0; I < LastOutput.ContactCount; ++I)
	{
		const auto& C = LastOutput.Contacts[I];
		DrawDebugPoint(World, C.Position, 6, FColor::Red);
		DrawDebugLine(World, C.Position, C.Position + C.Normal * 15, FColor::Red);
	}
	DrawDebugString(World, State.Position + FVector(0, 0, 15),
					FString::Printf(TEXT("F%llu %.1fus Q%u I%u"), State.SimulationFrame,
									LastOutput.Diagnostics.DurationMicroseconds, LastOutput.Diagnostics.Queries,
									LastOutput.Diagnostics.CollisionIterations),
					nullptr, FColor::White, 0);
}
