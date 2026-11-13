
#include "HandballSimulationComponent.h"
#include "Handball.h"
#include "HandballBarrier.h"
#include "HandballHandComponent.h"

#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Pawn.h"
#include "CollisionQueryParams.h"
#include "DrawDebugHelpers.h"

namespace
{
    FTransform InterpolatePose(
        const FTransform& A,
        const FTransform& B,
        float Alpha)
    {
        return FTransform(
            FQuat::Slerp(A.GetRotation(), B.GetRotation(), Alpha),
            FMath::Lerp(A.GetLocation(), B.GetLocation(), Alpha)
        );
    }

    // Signed sphere-to-box separation.
    // Positive = separated, negative = penetrating.
    float GetSphereBoxSeparation(
        const FVector& SpherePosition,
        const FTransform& BoxPose,
        const FVector& Extent,
        float Radius,
        FVector& OutNormal,
        FVector& OutLocalContact)
    {
        const FQuat Rotation = BoxPose.GetRotation();

        const FVector Local = Rotation.UnrotateVector(
            SpherePosition - BoxPose.GetLocation());

        const FVector Closest(
            FMath::Clamp(Local.X, -Extent.X, Extent.X),
            FMath::Clamp(Local.Y, -Extent.Y, Extent.Y),
            FMath::Clamp(Local.Z, -Extent.Z, Extent.Z)
        );

        const FVector Difference = Local - Closest;
        const float DistanceSq = Difference.SizeSquared();

        OutLocalContact = Closest;

        if (DistanceSq > UE_SMALL_NUMBER)
        {
            const float Distance = FMath::Sqrt(DistanceSq);

            OutNormal = Rotation.RotateVector(
                Difference / Distance);

            return Distance - Radius;
        }

        // Sphere centre is inside or on the box.
        const FVector FaceDistance(
            Extent.X - FMath::Abs(Local.X),
            Extent.Y - FMath::Abs(Local.Y),
            Extent.Z - FMath::Abs(Local.Z)
        );

        int32 Axis = 0;

        if (FaceDistance.Y < FaceDistance.X)
        {
            Axis = 1;
        }

        if (FaceDistance.Z < FaceDistance[Axis])
        {
            Axis = 2;
        }

        FVector LocalNormal = FVector::ZeroVector;
        LocalNormal[Axis] = Local[Axis] < 0.0f ? -1.0f : 1.0f;

        OutLocalContact = Local;
        OutLocalContact[Axis] = LocalNormal[Axis] * Extent[Axis];

        OutNormal = Rotation.RotateVector(LocalNormal);

        return -(FaceDistance[Axis] + Radius);
    }
}

UHandballSimulationComponent::UHandballSimulationComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.TickGroup = TG_PrePhysics;
}

void UHandballSimulationComponent::BeginPlay()
{
    Super::BeginPlay();

    // Find hands belonging to already-existing pawns.
    // Hands created later register themselves in BeginPlay.
    for (TActorIterator<APawn> It(GetWorld()); It; ++It)
    {
        TArray<UHandballHandComponent*> Hands;
        It->GetComponents<UHandballHandComponent>(Hands);

        for (UHandballHandComponent* Hand : Hands)
        {
            RegisterHand(Hand);
        }
    }
}

void UHandballSimulationComponent::RegisterHand(
    UHandballHandComponent* Hand)
{
    if (!IsValid(Hand))
    {
        return;
    }

    for (const FTrackedHand& Existing : TrackedHands)
    {
        if (Existing.Hand.Get() == Hand)
        {
            return;
        }
    }

    FTrackedHand NewHand;
    NewHand.Hand = Hand;
    NewHand.LastRenderPose = Hand->GetComponentTransform();
    NewHand.LastSimPose = NewHand.LastRenderPose;
    NewHand.CurrentPose = NewHand.LastRenderPose;

    TrackedHands.Add(NewHand);
}

void UHandballSimulationComponent::UnregisterHand(
    UHandballHandComponent* Hand)
{
    TrackedHands.RemoveAll(
        [Hand](const FTrackedHand& Entry)
        {
            return Entry.Hand.Get() == Hand;
        });
}

void UHandballSimulationComponent::TickComponent(
    float DeltaTime,
    ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    AHandball* Ball = Cast<AHandball>(GetOwner());

    if (!Ball || DeltaTime <= 0.0f)
    {
        return;
    }

    TrackedHands.RemoveAll(
        [](const FTrackedHand& Entry)
        {
            return !Entry.Hand.IsValid();
        });

    // Capture the latest hand poses once per render frame.
    for (FTrackedHand& Hand : TrackedHands)
    {
        Hand.CurrentPose = Hand.Hand->GetComponentTransform();
    }

    const float PreviousAccumulatedTime = AccumulatedTime;

    AccumulatedTime = FMath::Min(
        AccumulatedTime + DeltaTime,
        FixedTimeStep * MaxStepsPerFrame
    );

    FHandballState State = Ball->GetBallState();

    TArray<FHandballHandMotion> Motions;
    Motions.Reserve(TrackedHands.Num());

    int32 Steps = 0;

    while (AccumulatedTime >= FixedTimeStep &&
        Steps < MaxStepsPerFrame)
    {
        // Approximate the hand pose at this simulation boundary.
        const float StepEndTime =
            FixedTimeStep - PreviousAccumulatedTime +
            Steps * FixedTimeStep;

        const float Alpha = FMath::Clamp(
            StepEndTime / DeltaTime, 0.0f, 1.0f);

        Motions.Reset();

        for (FTrackedHand& Hand : TrackedHands)
        {
            const FTransform EndPose = InterpolatePose(
                Hand.LastRenderPose,
                Hand.CurrentPose,
                Alpha);

            FHandballHandMotion Motion;
            Motion.Start = Hand.LastSimPose;
            Motion.End = EndPose;
            Motion.Extent = Hand.Hand->GetScaledBoxExtent();

            Motions.Add(Motion);

            Hand.LastSimPose = EndPose;
        }

        SimulateStep(State, FixedTimeStep, Motions);

        AccumulatedTime -= FixedTimeStep;
        Steps++;
    }

    for (FTrackedHand& Hand : TrackedHands)
    {
        Hand.LastRenderPose = Hand.CurrentPose;
    }

    if (Steps > 0)
    {
        Ball->SetBallState(State);
    }

    if (bDebugRadius)
    {
        DrawDebugSphere(
            GetWorld(),
            Ball->GetBallState().Position,
            BallRadius,
            24,
            FColor::Green,
            false,
            0.0f,
            0,
            1.5f
        );
    }
}

bool UHandballSimulationComponent::FindBarrierHit(
    const FVector& Start,
    const FVector& End,
    FHitResult& OutHit) const
{
    UWorld* World = GetWorld();

    if (!World)
    {
        return false;
    }

    FCollisionQueryParams Params;
    Params.bTraceComplex = false;
    Params.AddIgnoredActor(GetOwner());

    const bool bHit = World->SweepSingleByChannel(
        OutHit,
        Start,
        End,
        FQuat::Identity,
        UEngineTypes::ConvertToCollisionChannel(
            BarrierTraceChannel.GetValue()),
        FCollisionShape::MakeSphere(BallRadius),
        Params
    );

    return bHit &&
        IsValid(Cast<AHandballBarrier>(OutHit.GetActor()));
}

bool UHandballSimulationComponent::FindHandHit(
    const FVector& Start,
    const FVector& End,
    const FHandballHandMotion& Motion,
    float Duration,
    FHandHit& OutHit) const
{
    if (Duration <= UE_SMALL_NUMBER)
    {
        return false;
    }

    // Broadphase: compare relative ball/hand-centre movement
    // against a sphere enclosing the palm box.
    const FVector RelativeStart =
        Start - Motion.Start.GetLocation();

    const FVector RelativeEnd =
        End - Motion.End.GetLocation();

    const FVector RelativeDelta = RelativeEnd - RelativeStart;
    const float DeltaSq = RelativeDelta.SizeSquared();

    const float ClosestAlpha = DeltaSq > UE_SMALL_NUMBER
        ? FMath::Clamp(
            -FVector::DotProduct(RelativeStart, RelativeDelta) /
            DeltaSq, 0.0, 1.0)
        : 0.0;

    const FVector ClosestRelative =
        RelativeStart + RelativeDelta * ClosestAlpha;

    const float BroadRadius = BallRadius + Motion.Extent.Size();

    if (ClosestRelative.SizeSquared() >
        FMath::Square(BroadRadius))
    {
        return false;
    }

    const float RotationTravel =
        Motion.Start.GetRotation().AngularDistance(
            Motion.End.GetRotation());

    const float MovementBound =
        RelativeDelta.Size() +
        RotationTravel * Motion.Extent.Size();

    constexpr int32 MaxIterations = 64;
    constexpr float ContactTolerance = 0.02f;

    float Alpha = 0.0f;

    for (int32 i = 0; i < MaxIterations; ++i)
    {
        const FVector SpherePosition =
            FMath::Lerp(Start, End, Alpha);

        const FTransform BoxPose = InterpolatePose(
            Motion.Start, Motion.End, Alpha);

        FVector Normal;
        FVector LocalContact;

        const float Separation = GetSphereBoxSeparation(
            SpherePosition,
            BoxPose,
            Motion.Extent,
            BallRadius,
            Normal,
            LocalContact
        );

        if (Separation <= ContactTolerance)
        {
            OutHit.Time = Alpha;
            OutHit.Normal = Normal.GetSafeNormal();
            OutHit.Penetration = FMath::Max(-Separation, 0.0f);

            // Velocity of the same palm contact point,
            // including translation and wrist rotation.
            const FVector PreviousPoint =
                Motion.Start.GetLocation() +
                Motion.Start.GetRotation().RotateVector(LocalContact);

            const FVector CurrentPoint =
                Motion.End.GetLocation() +
                Motion.End.GetRotation().RotateVector(LocalContact);

            OutHit.HandVelocity =
                (CurrentPoint - PreviousPoint) / Duration;

            return true;
        }

        if (MovementBound <= UE_SMALL_NUMBER)
        {
            return false;
        }

        // Conservative advancement towards first contact.
        const float Advance = Separation / MovementBound;

        if (Advance <= 0.0f || Alpha + Advance > 1.0f)
        {
            return false;
        }

        Alpha += Advance;
    }

    // Iteration budget exhausted. This needs explicit failure
    // handling before authoritative multiplayer use.
    return false;
}

void UHandballSimulationComponent::SimulateStep(
    FHandballState& State,
    float DeltaTime,
    const TArray<FHandballHandMotion>& HandMotions) const
{
    State.Velocity.Z += Gravity * DeltaTime;

    float RemainingTime = DeltaTime;

    constexpr int32 MaxImpactsPerStep = 6;
    constexpr float SurfaceOffset = 0.05f;

    for (int32 i = 0; i < MaxImpactsPerStep; ++i)
    {
        if (RemainingTime <= UE_SMALL_NUMBER)
        {
            break;
        }

        const FVector Start = State.Position;
        const FVector End =
            Start + State.Velocity * RemainingTime;

        FHitResult BarrierHit;
        const bool bBarrierHit =
            FindBarrierHit(Start, End, BarrierHit);

        float FirstHitTime = bBarrierHit
            ? FMath::Clamp(BarrierHit.Time, 0.0f, 1.0f)
            : 1.0f;

        bool bHandHit = false;
        FHandHit BestHandHit;

        // Account for the fraction of the step already consumed.
        const float MotionStartAlpha =
            1.0f - RemainingTime / DeltaTime;

        for (const FHandballHandMotion& Motion : HandMotions)
        {
            FHandballHandMotion RemainingMotion = Motion;

            RemainingMotion.Start = InterpolatePose(
                Motion.Start,
                Motion.End,
                MotionStartAlpha
            );

            FHandHit Candidate;

            if (FindHandHit(
                Start,
                End,
                RemainingMotion,
                RemainingTime,
                Candidate))
            {
                if ((!bBarrierHit || Candidate.Time < FirstHitTime) &&
                    (!bHandHit || Candidate.Time < BestHandHit.Time))
                {
                    BestHandHit = Candidate;
                    FirstHitTime = Candidate.Time;
                    bHandHit = true;
                }
            }
        }

        if (!bBarrierHit && !bHandHit)
        {
            State.Position = End;
            return;
        }

        FVector Normal;
        FVector SurfaceVelocity = FVector::ZeroVector;
        float Penetration = 0.0f;

        if (bHandHit)
        {
            Normal = BestHandHit.Normal;
            SurfaceVelocity = BestHandHit.HandVelocity;
            Penetration = BestHandHit.Penetration;

            State.Position =
                FMath::Lerp(Start, End, FirstHitTime);
        }
        else
        {
            Normal = BarrierHit.Normal.GetSafeNormal();

            if (BarrierHit.bStartPenetrating)
            {
                State.Position = Start;
                Penetration = BarrierHit.PenetrationDepth;
            }
            else
            {
                State.Position = BarrierHit.Location;
            }
        }

        if (Normal.IsNearlyZero())
        {
            State.Velocity = FVector::ZeroVector;
            return;
        }

        State.Position += Normal *
            (FMath::Max(Penetration, 0.0f) + SurfaceOffset);

        ResolveBounce(State, Normal, SurfaceVelocity);

        RemainingTime *= 1.0f - FirstHitTime;
    }
}

void UHandballSimulationComponent::ResolveBounce(
    FHandballState& State,
    const FVector& Normal,
    const FVector& SurfaceVelocity) const
{
    // The surface may itself be moving.
    const FVector RelativeVelocity =
        State.Velocity - SurfaceVelocity;

    const float NormalSpeed =
        FVector::DotProduct(RelativeVelocity, Normal);

    if (NormalSpeed >= 0.0f)
    {
        return;
    }

    const float BounceStrength =
        -NormalSpeed < MinBounceSpeed
        ? 0.0f
        : Restitution;

    State.Velocity -=
        (1.0f + BounceStrength) * NormalSpeed * Normal;
}
