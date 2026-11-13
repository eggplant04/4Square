#include "Handball.h"
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
}

void AHandball::SetBallState(const FHandballState& NewState)
{
    BallState = NewState;
    SetActorLocation(BallState.Position);
}