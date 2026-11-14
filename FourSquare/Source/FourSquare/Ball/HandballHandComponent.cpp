
#include "HandballHandComponent.h"
#include "Handball.h"
#include "MotionControllerComponent.h"
#include "HandballSimulationComponent.h"

#include "EngineUtils.h"

UHandballHandComponent::UHandballHandComponent()
{
    PrimaryComponentTick.bCanEverTick = false;

    InitBoxExtent(FVector(1.25f, 4.5f, 5.5f));

    SetCollisionEnabled(ECollisionEnabled::NoCollision);
    SetCollisionResponseToAllChannels(ECR_Ignore);
    SetGenerateOverlapEvents(false);
    SetSimulatePhysics(false);
}

bool UHandballHandComponent::TryGrabBall()
{
    if (!GetWorld())
    {
        return false;
    }

    for (TActorIterator<AHandball> It(GetWorld()); It; ++It)
    {
        if (UHandballSimulationComponent* Sim = It->GetSimulationComponent())
        {
            if (Sim->TryGrabBall(this))
            {
                return true;
            }
        }
    }

    return false;
}

void UHandballHandComponent::ReleaseBall()
{
    if (!GetWorld())
    {
        return;
    }

    for (TActorIterator<AHandball> It(GetWorld()); It; ++It)
    {
        if (UHandballSimulationComponent* Sim = It->GetSimulationComponent())
        {
            Sim->ReleaseBall(this);
        }
    }
}

bool UHandballHandComponent::HasValidTracking() const
{
    // Search the attachment hierarchy for our VR controller.
    for (const USceneComponent* Parent = GetAttachParent();
        Parent;
        Parent = Parent->GetAttachParent())
    {
        if (const UMotionControllerComponent* Controller =
            Cast<UMotionControllerComponent>(Parent))
        {
            return Controller->IsTracked();
        }
    }

    // Allow manually positioned hands for non-VR testing.
    return true;
}

void UHandballHandComponent::BeginPlay()
{
    Super::BeginPlay();

    for (TActorIterator<AHandball> It(GetWorld()); It; ++It)
    {
        if (UHandballSimulationComponent* Sim = It->GetSimulationComponent())
        {
            Sim->RegisterHand(this);
        }
    }
}

void UHandballHandComponent::EndPlay(
    const EEndPlayReason::Type EndPlayReason)
{
    for (TActorIterator<AHandball> It(GetWorld()); It; ++It)
    {
        if (UHandballSimulationComponent* Sim = It->GetSimulationComponent())
        {
            Sim->UnregisterHand(this);
        }
    }

    Super::EndPlay(EndPlayReason);
}
