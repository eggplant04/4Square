
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

    NewHand.PoseHistory.Reserve(32);

    FHandPoseSample InitialSample;
    InitialSample.TimeSeconds = GetWorld()->GetTimeSeconds();
    InitialSample.Pose = NewHand.CurrentPose;
    InitialSample.bValid = Hand->HasValidTracking();

    NewHand.bMotionValid = InitialSample.bValid;
    NewHand.PoseHistory.Add(InitialSample);

    TrackedHands.Add(MoveTemp(NewHand));
}

void UHandballSimulationComponent::UnregisterHand(
    UHandballHandComponent* Hand)
{
    if (HoldingHand.Get() == Hand)
    {
        HoldingHand.Reset();
        HeldLocalOffset = FVector::ZeroVector;
        AccumulatedTime = 0.0f;
    }

    TrackedHands.RemoveAll(
        [Hand](const FTrackedHand& Entry)
        {
            return Entry.Hand.Get() == Hand;
        });
}


bool UHandballSimulationComponent::TryGrabBall(
    UHandballHandComponent* Hand)
{
    if (!IsValid(Hand) || HoldingHand.IsValid())
    {
        return false;
    }

    AHandball* Ball = Cast<AHandball>(GetOwner());

    if (!Ball)
    {
        return false;
    }
    
    if (!IsValid(Hand) ||
        !Hand->HasValidTracking() ||
        HoldingHand.IsValid())
    {
        return false;
    }

    RegisterHand(Hand);

    const FTransform HandPose = Hand->GetComponentTransform();

    FVector Normal;
    FVector LocalContact;

    const float Separation = GetSphereBoxSeparation(
        Ball->GetBallState().Position,
        HandPose,
        Hand->GetScaledBoxExtent(),
        BallRadius,
        Normal,
        LocalContact
    );

    if (Separation > GrabReach)
    {
        return false;
    }

    HoldingHand = Hand;

    // Preserve the ball's position relative to the palm.
    // This prevents snapping when grabbing.
    HeldLocalOffset = HandPose.InverseTransformPositionNoScale(
        Ball->GetBallState().Position
    );

    for (FTrackedHand& Tracked : TrackedHands)
    {
        if (Tracked.Hand.Get() == Hand)
        {
            Tracked.LastRenderPose = HandPose;
            Tracked.LastSimPose = HandPose;
            Tracked.CurrentPose = HandPose;
            Tracked.ThrowVelocity = FVector::ZeroVector;
            break;
        }
    }

    FHandballState State = Ball->GetBallState();
    State.Velocity = FVector::ZeroVector;

    Ball->SetBallState(State);

    AccumulatedTime = 0.0f;

    return true;
}


void UHandballSimulationComponent::ReleaseBall(
    UHandballHandComponent* Hand)
{
    if (!IsValid(Hand) || HoldingHand.Get() != Hand)
    {
        return;
    }

    FTrackedHand* TrackedHand = nullptr;

    for (FTrackedHand& Tracked : TrackedHands)
    {
        if (Tracked.Hand.Get() == Hand)
        {
            TrackedHand = &Tracked;
            break;
        }
    }

    // Capture the release pose if a new timestamp is available.
    if (TrackedHand && GetWorld())
    {
        CaptureHandPose(
            *TrackedHand,
            GetWorld()->GetTimeSeconds()
        );
    }

    // Capturing may have detected tracking loss.
    if (HoldingHand.Get() != Hand)
    {
        return;
    }

    AHandball* Ball = Cast<AHandball>(GetOwner());

    if (Ball)
    {
        FHandballState State = Ball->GetBallState();
        State.Velocity = FVector::ZeroVector;

        if (TrackedHand)
        {
            if (TrackedHand->bMotionValid &&
                Hand->HasValidTracking() &&
                !TrackedHand->PoseHistory.IsEmpty())
            {
                const FTransform& ReleasePose =
                    TrackedHand->PoseHistory.Last().Pose;

                State.Position =
                    ReleasePose.TransformPositionNoScale(
                        HeldLocalOffset
                    );

                State.Velocity =
                    CalculateThrowVelocity(*TrackedHand) *
                    ThrowStrength;
            }

            TrackedHand->CooldownRemaining = HandHitCooldown;
            TrackedHand->bNeedsSeparation = true;
        }

        ClampBallSpeed(State);
        Ball->SetBallState(State);
    }

    HoldingHand.Reset();
    HeldLocalOffset = FVector::ZeroVector;
    AccumulatedTime = 0.0f;
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

    const double FrameTime = GetWorld()->GetTimeSeconds();

    // Record one timestamped pose per hand.
    for (FTrackedHand& Hand : TrackedHands)
    {
        CaptureHandPose(Hand, FrameTime);
    }

    if (HoldingHand.IsValid())
    {
        // While grabbed, follow the hand without running physics.
        FHandballState State = Ball->GetBallState();

        State.Position =
            HoldingHand->GetComponentTransform()
            .TransformPositionNoScale(HeldLocalOffset);

        State.Velocity = FVector::ZeroVector;

        Ball->SetBallState(State);

        AccumulatedTime = 0.0f;
    }
    else
    {
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
            // Absolute world times for this physics step.
            const double StepStartTime =
                FrameTime - AccumulatedTime;

            const double StepEndTime =
                StepStartTime + FixedTimeStep;

            Motions.Reset();

            for (int32 Index = 0;
                Index < TrackedHands.Num();
                ++Index)
            {
                FTrackedHand& Hand = TrackedHands[Index];

                if (!Hand.bMotionValid)
                {
                    continue;
                }

                FTransform StartPose;
                FTransform EndPose;

                if (!GetHandPoseAtTime(
                    Hand, StepStartTime, StartPose) ||
                    !GetHandPoseAtTime(
                        Hand, StepEndTime, EndPose))
                {
                    continue;
                }

                FHandballHandMotion Motion;
                Motion.Start = StartPose;
                Motion.End = EndPose;
                Motion.Extent =
                    Hand.Hand->GetScaledBoxExtent();

                Motion.TrackedHandIndex = Index;

                Motions.Add(Motion);
            }

            SimulateStep(State, FixedTimeStep, Motions);

            AccumulatedTime -= FixedTimeStep;
            Steps++;
        }

        if (Steps > 0)
        {
            Ball->SetBallState(State);
        }
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

void UHandballSimulationComponent::CaptureHandPose(
    FTrackedHand& Hand,
    double TimeSeconds)
{
    if (!Hand.Hand.IsValid())
    {
        return;
    }

    const FTransform NewPose =
        Hand.Hand->GetComponentTransform();

    bool bValid = Hand.Hand->HasValidTracking();

    if (!Hand.PoseHistory.IsEmpty())
    {
        const FHandPoseSample& Previous =
            Hand.PoseHistory.Last();

        const double Delta = TimeSeconds - Previous.TimeSeconds;

        // Do not invent movement when time hasn't advanced.
        if (Delta <= 0.00001)
        {
            return;
        }

        if (Delta > MaxHandSampleDelta)
        {
            bValid = false;
        }
        else if (bValid)
        {
            const double LinearSpeed =
                FVector::Distance(
                    Previous.Pose.GetLocation(),
                    NewPose.GetLocation()
                ) / Delta;

            const double AngularSpeed =
                FMath::RadiansToDegrees(
                    Previous.Pose.GetRotation().AngularDistance(
                        NewPose.GetRotation()
                    )
                ) / Delta;

            if (LinearSpeed > MaxHandTrackingSpeed ||
                AngularSpeed > MaxHandAngularSpeed)
            {
                bValid = false;
            }
        }
    }

    FHandPoseSample Sample;
    Sample.TimeSeconds = TimeSeconds;
    Sample.Pose = NewPose;
    Sample.bValid = bValid;

    Hand.PoseHistory.Add(Sample);
    Hand.CurrentPose = NewPose;
    Hand.bMotionValid = bValid;

    // Keep a short bounded history.
    const double OldestAllowed =
        TimeSeconds - HandPoseHistorySeconds;

    while (Hand.PoseHistory.Num() > 2 &&
        Hand.PoseHistory[1].TimeSeconds < OldestAllowed)
    {
        Hand.PoseHistory.RemoveAt(0);
    }

    if (!bValid)
    {
        Hand.bNeedsSeparation = true;
        Hand.CooldownRemaining = HandHitCooldown;

        // Tracking failure must never become a throw.
        Hand.ThrowVelocity = FVector::ZeroVector;

        if (HoldingHand.Get() == Hand.Hand.Get())
        {
            HoldingHand.Reset();
            HeldLocalOffset = FVector::ZeroVector;
            AccumulatedTime = 0.0f;
        }
    }
}

bool UHandballSimulationComponent::GetHandPoseAtTime(
    const FTrackedHand& Hand,
    double TimeSeconds,
    FTransform& OutPose) const
{
    const TArray<FHandPoseSample>& History = Hand.PoseHistory;

    if (History.IsEmpty())
    {
        return false;
    }

    constexpr double TimeTolerance = 0.00001;

    if (TimeSeconds < History[0].TimeSeconds - TimeTolerance ||
        TimeSeconds > History.Last().TimeSeconds + TimeTolerance)
    {
        return false;
    }

    // Exact recorded sample.
    for (const FHandPoseSample& Sample : History)
    {
        if (FMath::Abs(Sample.TimeSeconds - TimeSeconds)
            <= TimeTolerance)
        {
            if (!Sample.bValid)
            {
                return false;
            }

            OutPose = Sample.Pose;
            return true;
        }
    }

    // Find the two samples surrounding the requested time.
    for (int32 i = 1; i < History.Num(); ++i)
    {
        const FHandPoseSample& A = History[i - 1];
        const FHandPoseSample& B = History[i];

        if (TimeSeconds < A.TimeSeconds ||
            TimeSeconds > B.TimeSeconds)
        {
            continue;
        }

        const double Duration = B.TimeSeconds - A.TimeSeconds;

        if (!A.bValid || !B.bValid ||
            Duration <= 0.0 ||
            Duration > MaxHandSampleDelta)
        {
            return false;
        }

        const float Alpha = static_cast<float>(
            (TimeSeconds - A.TimeSeconds) / Duration
            );

        OutPose = InterpolatePose(
            A.Pose,
            B.Pose,
            Alpha
        );

        return true;
    }

    return false;
}

FVector UHandballSimulationComponent::CalculateThrowVelocity(
    const FTrackedHand& Hand) const
{
    if (Hand.PoseHistory.IsEmpty())
    {
        return FVector::ZeroVector;
    }

    const double EndTime =
        Hand.PoseHistory.Last().TimeSeconds;

    const double StartTime =
        EndTime - ThrowVelocityWindow;

    FTransform StartPose;
    FTransform EndPose;

    if (!GetHandPoseAtTime(Hand, StartTime, StartPose) ||
        !GetHandPoseAtTime(Hand, EndTime, EndPose))
    {
        return FVector::ZeroVector;
    }

    // Measure the actual held ball's movement,
    // including movement caused by wrist rotation.
    const FVector StartPosition =
        StartPose.TransformPositionNoScale(HeldLocalOffset);

    const FVector EndPosition =
        EndPose.TransformPositionNoScale(HeldLocalOffset);

    return (EndPosition - StartPosition) /
        (EndTime - StartTime);
}

void UHandballSimulationComponent::ClampBallSpeed(
    FHandballState& State) const
{
    if (MaxBallSpeed > 0.0f)
    {
        State.Velocity =
            State.Velocity.GetClampedToMaxSize(MaxBallSpeed);
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
    const TArray<FHandballHandMotion>& HandMotions)
{
    // Update each hand's independent cooldown.
    for (FTrackedHand& Hand : TrackedHands)
    {
        Hand.CooldownRemaining = FMath::Max(
            0.0f,
            Hand.CooldownRemaining - DeltaTime
        );
    }

    State.Velocity.Z += Gravity * DeltaTime;
    ClampBallSpeed(State);

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
        int32 BestHandIndex = INDEX_NONE;

        const float MotionStartAlpha =
            1.0f - RemainingTime / DeltaTime;

        for (const FHandballHandMotion& Motion : HandMotions)
        {
            if (!TrackedHands.IsValidIndex(Motion.TrackedHandIndex))
            {
                continue;
            }

            const FTrackedHand& Tracked =
                TrackedHands[Motion.TrackedHandIndex];

            // Ignore this hand during cooldown or until separated.
            if (Tracked.CooldownRemaining > 0.0f ||
                Tracked.bNeedsSeparation)
            {
                continue;
            }

            FHandballHandMotion RemainingMotion = Motion;

            RemainingMotion.Start = InterpolatePose(
                Motion.Start,
                Motion.End,
                MotionStartAlpha
            );

            FHandHit Candidate;

            if (!FindHandHit(
                Start,
                End,
                RemainingMotion,
                RemainingTime,
                Candidate))
            {
                continue;
            }

            // Only count approaching contacts as strikes.
            const FVector RelativeVelocity =
                State.Velocity -
                Candidate.HandVelocity * HandHitStrength;

            if (FVector::DotProduct(
                RelativeVelocity, Candidate.Normal) >= 0.0f)
            {
                continue;
            }

            if ((!bBarrierHit || Candidate.Time < FirstHitTime) &&
                (!bHandHit || Candidate.Time < BestHandHit.Time))
            {
                BestHandHit = Candidate;
                BestHandIndex = Motion.TrackedHandIndex;
                FirstHitTime = Candidate.Time;
                bHandHit = true;
            }
        }

        if (!bBarrierHit && !bHandHit)
        {
            State.Position = End;
            break;
        }

        FVector Normal;
        FVector SurfaceVelocity = FVector::ZeroVector;
        float Penetration = 0.0f;

        if (bHandHit)
        {
            Normal = BestHandHit.Normal;
            SurfaceVelocity =
                BestHandHit.HandVelocity * HandHitStrength;
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
            break;
        }

        State.Position += Normal *
            (FMath::Max(Penetration, 0.0f) + SurfaceOffset);

        if (bHandHit)
        {
            ResolveHandBounce(State, Normal, SurfaceVelocity);
        }
        else
        {
            ResolveBounce(State, Normal, SurfaceVelocity);
        }

        ClampBallSpeed(State);

        // A successful hand collision starts that hand's cooldown.
        if (bHandHit && TrackedHands.IsValidIndex(BestHandIndex))
        {
            FTrackedHand& Hand = TrackedHands[BestHandIndex];

            Hand.CooldownRemaining = HandHitCooldown;
            Hand.bNeedsSeparation = true;
        }

        RemainingTime *= 1.0f - FirstHitTime;
    }

    // Rearm hands once the ball has moved sufficiently away.
    for (const FHandballHandMotion& Motion : HandMotions)
    {
        if (!TrackedHands.IsValidIndex(Motion.TrackedHandIndex))
        {
            continue;
        }

        FTrackedHand& Hand =
            TrackedHands[Motion.TrackedHandIndex];

        if (!Hand.bNeedsSeparation)
        {
            continue;
        }

        FVector Normal;
        FVector LocalContact;

        const float Separation = GetSphereBoxSeparation(
            State.Position,
            Motion.End,
            Motion.Extent,
            BallRadius,
            Normal,
            LocalContact
        );

        if (Separation >= HandRearmDistance)
        {
            Hand.bNeedsSeparation = false;
        }
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


void UHandballSimulationComponent::ResolveHandBounce(
    FHandballState& State,
    const FVector& Normal,
    const FVector& HandVelocity) const
{
    // Check that the hand and ball are approaching each other.
    const FVector RelativeVelocity = State.Velocity - HandVelocity;

    if (FVector::DotProduct(RelativeVelocity, Normal) >= 0.0f)
    {
        return;
    }

    // First calculate the normal plate-style bounce.
    ResolveBounce(State, Normal, HandVelocity);

    if (HandDirectionInfluence <= 0.0f)
    {
        return;
    }

    const float Speed = State.Velocity.Size();

    if (Speed <= KINDA_SMALL_NUMBER || HandVelocity.IsNearlyZero())
    {
        return;
    }

    // Direction produced by the original bounce.
    const FVector PlateDirection = State.Velocity.GetSafeNormal();

    // Direction the palm was moving at contact.
    const FVector SwingDirection = HandVelocity.GetSafeNormal();

    const float Influence = FMath::Clamp(
        HandDirectionInfluence, 0.0f, 1.0f);

    // Blend the two directions.
    const FVector NewDirection = FMath::Lerp(
        PlateDirection,
        SwingDirection,
        Influence
    ).GetSafeNormal();

    if (!NewDirection.IsNearlyZero())
    {
        // Preserve the original ball speed.
        State.Velocity = NewDirection * Speed;
    }
}
