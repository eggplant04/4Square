#include "Ball/BallSimulationSubsystem.h"
#include "Ball/HandballBall.h"
#include "Ball/UnrealBallCollisionWorld.h"
#include "Engine/World.h"
#include "Ball/VRHandCollisionComponent.h"
#include "Ball/BallKinematicColliderComponent.h"
#include "Ball/HistoricalBallCollisionWorld.h"
#include "MotionControllerComponent.h"
#include "GameFramework/Pawn.h"
#include "EngineUtils.h"
#include "HAL/PlatformTime.h"
#include "ProfilingDebugging/CpuProfilerTrace.h"

void FBallSimulationTickFunction::ExecuteTick(float Delta, ELevelTick Type, ENamedThreads::Type Thread,
											  const FGraphEventRef &Completion)
{
	if (Target)
	{
		Target->TickSimulation(Delta);
	}
}

UBallSimulationSubsystem::~UBallSimulationSubsystem() = default;
void UBallSimulationSubsystem::Initialize(FSubsystemCollectionBase &Collection)
{
	Super::Initialize(Collection);
	Balls.Reserve(16);
	HandSources.Reserve(BallMotionLimits::Hands / 2);
	KinematicSources.Reserve(BallMotionLimits::Kinematics);
	TrackingTimeOrigin = GetWorld()->GetTimeSeconds();
	LastWorldOrigin = GetWorld()->OriginLocation;
	Epoch = FGuid::NewGuid();
	Clock.Initialize({}, EBallClockRole::Authority);
	TickFunction.Target = this;
	TickFunction.bCanEverTick = true;
	TickFunction.bStartWithTickEnabled = true;
	TickFunction.TickGroup = TG_PostPhysics;
	TickFunction.EndTickGroup = TG_PostPhysics;
}
bool UBallSimulationSubsystem::DoesSupportWorldType(EWorldType::Type Type) const
{
	return Type == EWorldType::Game || Type == EWorldType::PIE;
}
void UBallSimulationSubsystem::Deinitialize()
{
	TickFunction.UnRegisterTickFunction();
	TickFunction.Target = nullptr;
	CollisionWorld.Reset();
	Balls.Reset();
	HandSources.Reset();
	KinematicSources.Reset();
	MotionHistory.Reset();
	Super::Deinitialize();
}
uint32 UBallSimulationSubsystem::GetEnvironmentRevision() const
{
	return CollisionWorld ? CollisionWorld->GetRevision() : 0;
}

FString UBallSimulationSubsystem::GetSimulationDiagnostics() const
{
	return FString::Printf(
		TEXT("Frame=%llu TickRegistered=%d TickEnabled=%d Status=%d Debt=%.6f Overruns=%llu RenderFrames=%llu "
			 "StartupWaiting=%d Hands=%d Kinematics=%d MotionBytes=%llu MotionBuild=%.2fus Peak=%.2fus"),
		Clock.GetFrame(), TickFunction.IsTickFunctionRegistered(), TickFunction.IsTickFunctionEnabled(),
		static_cast<int32>(Clock.GetStatus()), Clock.GetAccumulatorSeconds(), Clock.GetDiagnostics().Overruns,
		Clock.GetDiagnostics().RenderFrames, bWaitingForStartup, HandSources.Num(), KinematicSources.Num(),
		uint64(MotionHistory.GetAllocatedBytes()), LastMotionBuildMicroseconds, PeakMotionBuildMicroseconds);
}

double UBallSimulationSubsystem::GetTrackingTimelineSeconds() const
{
	return GetWorld()->GetTimeSeconds() - TrackingTimeOrigin;
}
void UBallSimulationSubsystem::ResetMotionTimebase()
{
	MotionHistory.Reset();
	TrackingTimeOrigin = GetWorld()->GetTimeSeconds() - Clock.GetFrame() / Frequency;
}
uint32 UBallSimulationSubsystem::RegisterHandSource(UVRHandCollisionComponent *Source)
{
	if (!IsValid(Source) || Source->GetWorld() != GetWorld() || NextParticipantId == MAX_uint32)
	{
		return 0;
	}
	const auto *Pawn = Cast<APawn>(Source->GetOwner());
	if (!Source->bAllowScriptedTrackingForTests &&
		(!Pawn || !Pawn->IsPlayerControlled() || !Pawn->IsLocallyControlled()))
	{
		return 0;
	}
	HandSources.RemoveAll([](const auto &P) { return !P.IsValid(); });
	for (const auto &P : HandSources)
	{
		if (P.Get() == Source)
		{
			return Source->ParticipantId;
		}
		if (P->GetOwner() == Source->GetOwner())
		{
			return 0;
		}
	}
	if (HandSources.Num() >= BallMotionLimits::Hands / 2)
	{
		return 0;
	}
	Source->ParticipantId = NextParticipantId++;
	HandSources.Add(Source);
	TickFunction.AddPrerequisite(Source, Source->PrimaryComponentTick);
	return Source->ParticipantId;
}
void UBallSimulationSubsystem::UnregisterHandSource(UVRHandCollisionComponent *Source)
{
	if (Source)
	{
		TickFunction.RemovePrerequisite(Source, Source->PrimaryComponentTick);
	}
	HandSources.RemoveAll([&](const auto &P) { return !P.IsValid() || P.Get() == Source; });
}
uint32 UBallSimulationSubsystem::RegisterKinematicSource(UBallKinematicColliderComponent *Source)
{
	if (!IsValid(Source) || Source->GetWorld() != GetWorld() || NextKinematicId == MAX_uint32)
	{
		return 0;
	}
	KinematicSources.RemoveAll([](const auto &P) { return !P.IsValid(); });
	for (const auto &P : KinematicSources)
	{
		if (P.Get() == Source)
		{
			return Source->SourceId;
		}
	}
	if (KinematicSources.Num() >= BallMotionLimits::Kinematics)
	{
		return 0;
	}
	Source->SourceId = NextKinematicId++;
	KinematicSources.Add(Source);
	TickFunction.AddPrerequisite(Source, Source->PrimaryComponentTick);
	return Source->SourceId;
}
void UBallSimulationSubsystem::UnregisterKinematicSource(UBallKinematicColliderComponent *Source)
{
	if (Source)
	{
		TickFunction.RemovePrerequisite(Source, Source->PrimaryComponentTick);
	}
	KinematicSources.RemoveAll([&](const auto &P) { return !P.IsValid() || P.Get() == Source; });
}
void UBallSimulationSubsystem::DiscoverLocalHands()
{
	bool Enabled = false;
	for (const auto &B : Balls)
	{
		Enabled |= B.IsValid() && B->bAutoRegisterVRHands;
	}
	if (!Enabled)
	{
		return;
	}
	for (TActorIterator<APawn> It(GetWorld()); It; ++It)
	{
		auto *Pawn = *It;
		if (!Pawn->IsPlayerControlled() || !Pawn->IsLocallyControlled() ||
			Pawn->FindComponentByClass<UVRHandCollisionComponent>())
		{
			continue;
		}
		UMotionControllerComponent *L = nullptr, *R = nullptr;
		if (!UVRHandCollisionComponent::FindUniqueGrips(Pawn, L, R))
		{
			continue;
		}
		auto *Component = NewObject<UVRHandCollisionComponent>(Pawn);
		Pawn->AddInstanceComponent(Component);
		Component->RegisterComponent();
	}
}

bool UBallSimulationSubsystem::RegisterBall(AHandballBall *Ball)
{
	if (!IsValid(Ball) || Ball->GetWorld() != GetWorld() || GetWorld()->GetNetMode() == NM_Client ||
		!Ball->Physics.IsValid())
	{
		return false;
	}
	for (const auto &Entry : Balls)
	{
		if (Entry.Get() == Ball)
		{
			return true;
		}
	}
	if (Balls.Num() == 16)
	{
		return false;
	}
	if (Balls.Num() == 0 && Frequency != Ball->Physics.FixedFrequency)
	{
		Frequency = Ball->Physics.FixedFrequency;
		Epoch = FGuid::NewGuid();
		FBallClockSettings Settings;
		Settings.FixedFrequency = Frequency;
		Clock.Initialize(Settings, EBallClockRole::Authority);
	}
	if (Frequency != Ball->Physics.FixedFrequency)
	{
		return false;
	}
	if (MotionHistory.GetCapacity() < Ball->HistorySettings.HistoryCapacity)
	{
		if (Clock.GetFrame() != 0 && MotionHistory.GetCapacity() != 0)
		{
			return false;
		}
		if (!MotionHistory.Initialize(Ball->HistorySettings.HistoryCapacity))
		{
			return false;
		}
	}
	if (Balls.Num() == 0)
	{
		if (Clock.GetFrame() > 0 && Clock.GetTimebase() < MAX_uint32)
		{
			Clock.PauseAuthority();
			Clock.ResumeAuthority(Clock.GetTimebase() + 1);
		}
		ResetMotionTimebase();
	}
	if (!CollisionWorld)
	{
		CollisionWorld = MakeUnique<FUnrealBallCollisionWorld>();
		if (!CollisionWorld->Initialize(GetWorld(), EnvironmentRevision))
		{
			CollisionWorld.Reset();
			return false;
		}
	}
	if (!Ball->InitializeSimulation({Epoch, NextBallId++, 1}, Clock.GetFrame(), GetEnvironmentRevision()))
	{
		return false;
	}
	Balls.Add(Ball);
	if (!TickFunction.IsTickFunctionRegistered())
	{
		TickFunction.RegisterTickFunction(GetWorld()->PersistentLevel);
	}
	return true;
}

void UBallSimulationSubsystem::UnregisterBall(AHandballBall *Ball)
{
	Balls.RemoveAll([&](const auto &Entry) { return !Entry.IsValid() || Entry.Get() == Ball; });
}

void UBallSimulationSubsystem::TickSimulation(double Elapsed)
{
	Balls.RemoveAll([](const auto &Entry) { return !Entry.IsValid(); });
	HandSources.RemoveAll([](const auto &Entry) { return !Entry.IsValid(); });
	KinematicSources.RemoveAll([](const auto &Entry) { return !Entry.IsValid(); });
	if (Balls.Num() == 0)
	{
		return;
	}
	if (GetWorld()->OriginLocation != LastWorldOrigin)
	{
		if (!RebuildStaticEnvironment())
		{
			Clock.PauseAuthority();
			return;
		}
		LastWorldOrigin = GetWorld()->OriginLocation;
	}
	DiscoverLocalHands();
	if (bWaitingForStartup)
	{
		// There is no simulated/committed gameplay yet. Count this declared startup pause,
		// then establish a new frame-zero timebase after three frames fitting the catch-up budget.
		Clock.BeginRenderFrame(Elapsed);
		const bool bReady = FMath::IsFinite(Elapsed) && Elapsed >= 0 &&
							Elapsed <= Clock.GetFixedDeltaTime() * Clock.GetMaximumCatchUpSteps();
		StableStartupFrames = bReady ? StableStartupFrames + 1 : 0;
		if (StableStartupFrames >= 3 && GetEnvironmentRevision() != 0 && Clock.GetTimebase() < MAX_uint32)
		{
			Clock.ResumeAuthority(Clock.GetTimebase() + 1);
			ResetMotionTimebase();
			bWaitingForStartup = false;
			UE_LOG(LogHandballBall, Display,
				   TEXT("Ball startup ready: frame-zero timebase %u; explicitly recorded startup pause %.6fs."),
				   Clock.GetTimebase(), Clock.GetDiagnostics().ExplicitlyDiscardedSeconds);
		}
		return;
	}
	if (Clock.GetFrame() == 0 && Clock.GetDiagnostics().RenderFrames == 0 && FMath::IsFinite(Elapsed) &&
		Elapsed > Clock.GetFixedDeltaTime() * Clock.GetMaximumCatchUpSteps())
	{
		// A first frame can fit the backlog limit yet already exceed the catch-up budget.
		// Declare startup before publishing even one gameplay step, rather than freezing at frame eight.
		Clock.BeginRenderFrame(Elapsed);
		Clock.PauseAuthority();
		bWaitingForStartup = true;
		StableStartupFrames = 0;
		UE_LOG(LogHandballBall, Display,
			   TEXT("Ball startup waiting: first render interval %.6fs exceeds catch-up budget; frame-zero state "
					"preserved."),
			   Elapsed);
		return;
	}
	if (GetEnvironmentRevision() == 0 && Clock.GetStatus() == EBallClockStatus::Running)
	{
		Clock.PauseAuthority();
		UE_LOG(LogHandballBall, Warning,
			   TEXT("Static environment changed; paused. Call RebuildStaticEnvironment at an explicit boundary."));
	}
	if (!Clock.BeginRenderFrame(Elapsed))
	{
		if (Clock.GetStatus() == EBallClockStatus::AuthorityPaused && Clock.GetFrame() == 0 &&
			Clock.GetDiagnostics().Overruns > 0 && !bFaultReported)
		{
			bWaitingForStartup = true;
			StableStartupFrames = 0;
			UE_LOG(LogHandballBall, Display,
				   TEXT("Ball frame-zero timeline waiting for startup readiness after %.6fs backlog; no gameplay "
						"advanced."),
				   Clock.GetAccumulatorSeconds());
		}
		else if (!bFaultReported)
		{
			UE_LOG(LogHandballBall, Warning, TEXT("Ball simulation paused; explicit recovery required. %s"),
				   *GetSimulationDiagnostics());
			bFaultReported = true;
		}
		return;
	}
	while (Clock.HasStepAvailable())
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(Handball_FixedWorldTransaction);
		const double BuildStart = FPlatformTime::Seconds();
		FBallMotionFrame Motion;
		Motion.Frame = Clock.GetFrame();
		Motion.Epoch = Epoch;
		Motion.Timebase = Clock.GetTimebase();
		Motion.EnvironmentRevision = GetEnvironmentRevision();
		Motion.DeltaTime = Clock.GetFixedDeltaTime();
		for (const auto &Source : HandSources)
		{
			if (Source.IsValid())
			{
				Source->BuildRecordedMotion(Motion, Motion.Frame / Frequency);
			}
		}
		for (const auto &Source : KinematicSources)
		{
			if (Source.IsValid())
			{
				Source->AppendRecordedMotion(Motion, Motion.Frame / Frequency);
			}
		}
		LastMotionBuildMicroseconds = (FPlatformTime::Seconds() - BuildStart) * 1.e6;
		PeakMotionBuildMicroseconds = FMath::Max(PeakMotionBuildMicroseconds, LastMotionBuildMicroseconds);
		if (!MotionHistory.CanPush(Motion))
		{
			Clock.PauseAuthority();
			UE_LOG(LogHandballBall, Warning,
				   TEXT("Motion history transaction rejected: pinned capacity or invalid trajectory."));
			break;
		}
		FHistoricalBallCollisionWorld HistoricalWorld(*CollisionWorld, MotionHistory, &Motion);
		bool bReady = true;
		for (const auto &Entry : Balls)
		{
			bReady &= Entry->PrepareStep(HistoricalWorld, Clock.GetFrame(), Motion.Timebase, Motion.Revision);
		}
		if (!bReady)
		{
			Clock.PauseAuthority();
			if (!bFaultReported)
			{
				UE_LOG(LogHandballBall, Warning,
					   TEXT("Ball transaction failed; world timeline paused without partial state advancement."));
				bFaultReported = true;
			}
			break;
		}
		if (!MotionHistory.Push(Motion))
		{
			UE_LOG(LogHandballBall, Fatal, TEXT("Preflighted motion history write failed."));
		}
		for (const auto &Entry : Balls)
		{
			Entry->CommitStep();
		}
		Clock.ConsumeCompletedStep();
	}
	for (const auto &Entry : Balls)
	{
		Entry->DrawDiagnostics();
	}
}

bool UBallSimulationSubsystem::RebuildStaticEnvironment()
{
	if (GetWorld()->GetNetMode() == NM_Client || EnvironmentRevision == MAX_uint32 || Clock.GetTimebase() == MAX_uint32)
	{
		return false;
	}
	for (const auto &Entry : Balls)
	{
		if (!Entry.IsValid() || !Entry->Physics.IsValid() || Entry->Physics.FixedFrequency != Frequency ||
			!Entry->HistorySettings.IsValid(Frequency) || Entry->State.Identity.Generation == MAX_uint32 ||
			!Entry->State.IsValid())
		{
			return false;
		}
	}
	Clock.PauseAuthority();
	if (!CollisionWorld)
	{
		CollisionWorld = MakeUnique<FUnrealBallCollisionWorld>();
	}
	if (!CollisionWorld->Initialize(GetWorld(), ++EnvironmentRevision))
	{
		return false;
	}
	for (const auto &Entry : Balls)
	{
		if (Entry->State.Identity.Generation == MAX_uint32)
		{
			return false;
		}
		auto Id = Entry->State.Identity;
		++Id.Generation;
		if (!Entry->InitializeSimulation(Id, Clock.GetFrame(), EnvironmentRevision))
		{
			return false;
		}
	}
	bFaultReported = false;
	bWaitingForStartup = false;
	StableStartupFrames = 0;
	const bool Resumed = Clock.ResumeAuthority(Clock.GetTimebase() + 1);
	ResetMotionTimebase();
	LastWorldOrigin = GetWorld()->OriginLocation;
	return Resumed;
}
