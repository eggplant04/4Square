#include "Ball/BallTestCourt.h"
#include "Components/SceneComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "UObject/ConstructorHelpers.h"

ABallTestCourt::ABallTestCourt()
{
	PrimaryActorTick.bCanEverTick = false;
	auto* Root = CreateDefaultSubobject<USceneComponent>(TEXT("CourtRoot"));
	Root->SetMobility(EComponentMobility::Static);
	SetRootComponent(Root);
	static ConstructorHelpers::FObjectFinder<UStaticMesh> Cube(TEXT("/Engine/BasicShapes/Cube.Cube"));
	const auto Box = [&](const TCHAR* Name, FVector Center, FVector Extent)
	{
		auto* Component = CreateDefaultSubobject<UStaticMeshComponent>(Name);
		Component->SetupAttachment(Root);
		Component->SetMobility(EComponentMobility::Static);
		Component->SetRelativeLocation(Center);
		Component->SetRelativeScale3D(Extent / 50.0);
		Component->SetCollisionProfileName(TEXT("BlockAll"));
		Component->SetSimulatePhysics(false);
		if (Cube.Succeeded())
		{
			Component->SetStaticMesh(Cube.Object);
		}
		return Component;
	};
	Box(TEXT("Floor"), FVector(0, 0, -10), FVector(1000, 500, 10));
	Box(TEXT("WallXPositive"), FVector(1010, 0, 500), FVector(10, 520, 500));
	Box(TEXT("WallXNegative"), FVector(-1010, 0, 500), FVector(10, 520, 500));
	Box(TEXT("WallYPositive"), FVector(0, 510, 500), FVector(1000, 10, 500));
	Box(TEXT("WallYNegative"), FVector(0, -510, 500), FVector(1000, 10, 500));
	Ceiling = Box(TEXT("Ceiling"), FVector(0, 0, 1010), FVector(1000, 500, 10));
}
void ABallTestCourt::OnConstruction(const FTransform& Transform)
{
	Super::OnConstruction(Transform);
	Ceiling->SetCollisionEnabled(bIncludeCeiling ? ECollisionEnabled::QueryOnly : ECollisionEnabled::NoCollision);
}
