
#include "Handball.h"
#include "HandballSimulationComponent.h"

#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Net/UnrealNetwork.h"

AHandball::AHandball()
{
    PrimaryActorTick.bCanEverTick = false;

    // Networking
    bReplicates = true;
    SetReplicateMovement(false);
    bAlwaysRelevant = true;

    NetUpdateFrequency = 60.0f;
    MinNetUpdateFrequency = 30.0f;

    // Components
    SceneRoot = CreateDefaultSubobject<USceneComponent>(
        TEXT("SceneRoot")
    );

    RootComponent = SceneRoot;

    BallMesh = CreateDefaultSubobject<UStaticMeshComponent>(
        TEXT("BallMesh")
    );

    BallMesh->SetupAttachment(SceneRoot);

    BallMesh->SetSimulatePhysics(false);
    BallMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    BallMesh->SetGenerateOverlapEvents(false);

    SimulationComponent =
        CreateDefaultSubobject<UHandballSimulationComponent>(
            TEXT("SimulationComponent")
        );
}

void AHandball::BeginPlay()
{
    Super::BeginPlay();

    if (HasAuthority())
    {
        // Only the server initialises the authoritative state.
        BallState.Position = GetActorLocation();
        BallState.Velocity = FVector::ZeroVector;
    }
    else
    {
        // Clients receive their state from the server.
        SimulationComponent->SetComponentTickEnabled(false);
    }

    SpeedMaterial = BallMesh->CreateDynamicMaterialInstance(0);

    UpdateSpeedMaterial();
}

void AHandball::GetLifetimeReplicatedProps(
    TArray<FLifetimeProperty>& OutLifetimeProps) const
{
    Super::GetLifetimeReplicatedProps(OutLifetimeProps);

    DOREPLIFETIME(AHandball, BallState);
}

void AHandball::SetBallState(const FHandballState& NewState)
{
    // Only the server can change the official ball state.
    if (!HasAuthority())
    {
        return;
    }

    BallState = NewState;

    ApplyBallState();
}

void AHandball::OnRep_BallState()
{
    // Called on clients when the server sends a new state.
    ApplyBallState();
}

void AHandball::ApplyBallState()
{
    SetActorLocation(BallState.Position);

    UpdateSpeedMaterial();
}

void AHandball::UpdateSpeedMaterial()
{
    if (!SpeedMaterial || !SimulationComponent)
    {
        return;
    }

    const float MaxSpeed =
        SimulationComponent->GetMaxBallSpeed();

    const float SpeedAlpha = MaxSpeed > 0.0f
        ? FMath::Clamp(
            BallState.Velocity.Size() / MaxSpeed,
            0.0f,
            1.0f
        )
        : 0.0f;

    SpeedMaterial->SetScalarParameterValue(
        TEXT("SpeedAlpha"),
        SpeedAlpha
    );
}
