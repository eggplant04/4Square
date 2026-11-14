
#include "TestPawn.h"
#include "FourSquare/Ball/HandballHandComponent.h"

#include "Camera/CameraComponent.h"
#include "Components/CapsuleComponent.h"
#include "Components/InputComponent.h"
#include "Components/SkeletalMeshComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "GameFramework/CharacterMovementComponent.h"
#include "GameFramework/Controller.h"
#include "GameFramework/SpringArmComponent.h"
#include "UObject/ConstructorHelpers.h"

ATestPawn::ATestPawn()
{
    PrimaryActorTick.bCanEverTick = false;

    // Networking
    bReplicates = true;
    SetReplicateMovement(true);

    // Character rotation
    bUseControllerRotationYaw = true;
    bUseControllerRotationPitch = false;
    bUseControllerRotationRoll = false;

    // Flying movement
    UCharacterMovementComponent* Movement = GetCharacterMovement();

    Movement->DefaultLandMovementMode = MOVE_Flying;
    Movement->GravityScale = 0.0f;
    Movement->MaxFlySpeed = 900.0f;
    Movement->MaxAcceleration = 8000.0f;
    Movement->BrakingDecelerationFlying = 3000.0f;
    Movement->bOrientRotationToMovement = false;
    Movement->bEnablePhysicsInteraction = false;

    // Ignore all world collision while retaining query-enabled
    // capsule geometry for CharacterMovement.
    GetCapsuleComponent()->SetCollisionEnabled(
        ECollisionEnabled::QueryOnly
    );
    GetCapsuleComponent()->SetCollisionResponseToAllChannels(
        ECR_Ignore
    );
    GetCapsuleComponent()->SetGenerateOverlapEvents(false);

    // Hide unused inherited skeletal mesh.
    GetMesh()->SetVisibility(false);
    GetMesh()->SetCollisionEnabled(ECollisionEnabled::NoCollision);

    // Visual head
    HeadMesh = CreateDefaultSubobject<UStaticMeshComponent>(
        TEXT("HeadMesh")
    );

    HeadMesh->SetupAttachment(RootComponent);
    HeadMesh->SetSimulatePhysics(false);
    HeadMesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    HeadMesh->SetGenerateOverlapEvents(false);
    HeadMesh->SetRelativeScale3D(FVector(0.3f));

    static ConstructorHelpers::FObjectFinder<UStaticMesh> CubeMesh(
        TEXT("/Engine/BasicShapes/Cube.Cube")
    );

    if (CubeMesh.Succeeded())
    {
        HeadMesh->SetStaticMesh(CubeMesh.Object);
    }

    // Custom handball collision proxy
    HeadCollider = CreateDefaultSubobject<UHandballHandComponent>(
        TEXT("HeadCollider")
    );

    HeadCollider->SetupAttachment(RootComponent);
    HeadCollider->SetCollisionEnabled(ECollisionEnabled::NoCollision);
    HeadCollider->SetGenerateOverlapEvents(false);

    // Third-person camera
    CameraBoom = CreateDefaultSubobject<USpringArmComponent>(
        TEXT("CameraBoom")
    );

    CameraBoom->SetupAttachment(RootComponent);
    CameraBoom->TargetArmLength = 300.0f;
    CameraBoom->SetRelativeLocation(FVector(0.0f, 0.0f, 40.0f));
    CameraBoom->bUsePawnControlRotation = true;
    CameraBoom->bDoCollisionTest = false;

    FollowCamera = CreateDefaultSubobject<UCameraComponent>(
        TEXT("FollowCamera")
    );

    FollowCamera->SetupAttachment(
        CameraBoom,
        USpringArmComponent::SocketName
    );

    FollowCamera->bUsePawnControlRotation = false;
}

void ATestPawn::BeginPlay()
{
    Super::BeginPlay();

    if (HasAuthority())
    {
        GetCharacterMovement()->SetMovementMode(MOVE_Flying);
    }
}

void ATestPawn::OnConstruction(const FTransform& Transform)
{
    Super::OnConstruction(Transform);

    if (!HeadMesh || !HeadCollider)
    {
        return;
    }

    if (const UStaticMesh* StaticMesh = HeadMesh->GetStaticMesh())
    {
        HeadCollider->SetBoxExtent(StaticMesh->GetBounds().BoxExtent);
        HeadCollider->SetRelativeTransform(
            HeadMesh->GetRelativeTransform()
        );
    }
}

void ATestPawn::SetupPlayerInputComponent(
    UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);

    PlayerInputComponent->BindAxis(
        "HB_MoveForward", this, &ATestPawn::MoveForward);

    PlayerInputComponent->BindAxis(
        "HB_MoveRight", this, &ATestPawn::MoveRight);

    PlayerInputComponent->BindAxis(
        "HB_MoveUp", this, &ATestPawn::MoveUp);

    PlayerInputComponent->BindAxis(
        "HB_Turn", this, &APawn::AddControllerYawInput);

    PlayerInputComponent->BindAxis(
        "HB_LookUp", this, &APawn::AddControllerPitchInput);
}

void ATestPawn::MoveForward(float Value)
{
    if (!Controller || FMath::IsNearlyZero(Value))
    {
        return;
    }

    const FRotator YawRotation(
        0.0f,
        Controller->GetControlRotation().Yaw,
        0.0f
    );

    AddMovementInput(
        FRotationMatrix(YawRotation).GetUnitAxis(EAxis::X),
        Value
    );
}

void ATestPawn::MoveRight(float Value)
{
    if (!Controller || FMath::IsNearlyZero(Value))
    {
        return;
    }

    const FRotator YawRotation(
        0.0f,
        Controller->GetControlRotation().Yaw,
        0.0f
    );

    AddMovementInput(
        FRotationMatrix(YawRotation).GetUnitAxis(EAxis::Y),
        Value
    );
}

void ATestPawn::MoveUp(float Value)
{
    if (!FMath::IsNearlyZero(Value))
    {
        AddMovementInput(FVector::UpVector, Value);
    }
}
