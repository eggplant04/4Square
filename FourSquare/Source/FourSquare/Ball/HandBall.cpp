#include "Handball.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "HandballSimulationComponent.h"

AHandball::AHandball()
{
    PrimaryActorTick.bCanEverTick = false;

    SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot"));
    RootComponent = SceneRoot;

    BallMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BallMesh"));
    BallMesh->SetupAttachment(SceneRoot);

    BallMesh->SetSimulatePhysics(false);
    BallMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    BallMesh->SetGenerateOverlapEvents(false);

    SimulationComponent = CreateDefaultSubobject<UHandballSimulationComponent>(TEXT("SimulationComponent"));
}

void AHandball::BeginPlay()
{
    Super::BeginPlay();

    BallState.Position = GetActorLocation();
    BallState.Velocity = FVector::ZeroVector;

    SpeedMaterial = BallMesh->CreateDynamicMaterialInstance(0);
    UpdateSpeedMaterial();
}

void AHandball::UpdateSpeedMaterial()
{
    if (!SpeedMaterial || !SimulationComponent)
    {
        return;
    }

    const float MaxSpeed = SimulationComponent->GetMaxBallSpeed();

    const float SpeedAlpha = MaxSpeed > 0.0f
        ? FMath::Clamp(BallState.Velocity.Size() / MaxSpeed, 0.0f, 1.0f)
        : 0.0f;

    SpeedMaterial->SetScalarParameterValue(
        TEXT("SpeedAlpha"),
        SpeedAlpha
    );
}

void AHandball::SetBallState(const FHandballState& NewState)
{
    BallState = NewState;

    SetActorLocation(BallState.Position);

    UpdateSpeedMaterial();
}