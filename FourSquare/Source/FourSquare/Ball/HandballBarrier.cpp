#include "HandballBarrier.h"
#include "Components/StaticMeshComponent.h"

AHandballBarrier::AHandballBarrier()
{
    PrimaryActorTick.bCanEverTick = false;

    BarrierMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("BarrierMesh"));
    RootComponent = BarrierMesh;

    BarrierMesh->SetMobility(EComponentMobility::Static);
    BarrierMesh->SetSimulatePhysics(false);
    BarrierMesh->SetCollisionEnabled(ECollisionEnabled::QueryOnly);
    BarrierMesh->SetCollisionResponseToAllChannels(ECR_Ignore);
}