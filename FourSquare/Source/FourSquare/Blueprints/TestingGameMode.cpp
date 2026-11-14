
#include "TestingGameMode.h"
#include "GameFramework/Controller.h"

UClass* ATestingGameMode::GetDefaultPawnClassForController_Implementation(
    AController* InController)
{
    // The listen server's local player uses VR.
    if (InController && InController->IsLocalController())
    {
        if (VRPawnClass)
        {
            return VRPawnClass.Get();
        }
    }
    else
    {
        // Remote players use our desktop testing pawn.
        if (TestPawnClass)
        {
            return TestPawnClass.Get();
        }
    }

    return Super::GetDefaultPawnClassForController_Implementation(InController);
}
