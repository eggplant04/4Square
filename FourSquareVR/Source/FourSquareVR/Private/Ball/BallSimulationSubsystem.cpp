#include "Ball/BallSimulationSubsystem.h"
#include "Ball/HandballBall.h"
#include "Ball/UnrealBallCollisionWorld.h"
#include "Engine/World.h"

void FBallSimulationTickFunction::ExecuteTick(float Delta, ELevelTick Type, ENamedThreads::Type Thread,
											  const FGraphEventRef& Completion)
{
	if (Target)
	{
		Target->TickSimulation(Delta);
	}
}

UBallSimulationSubsystem::~UBallSimulationSubsystem() = default;
void UBallSimulationSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
	Balls.Reserve(16);
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
	Super::Deinitialize();
}
uint32 UBallSimulationSubsystem::GetEnvironmentRevision() const
{
	return CollisionWorld ? CollisionWorld->GetRevision() : 0;
}

bool UBallSimulationSubsystem::RegisterBall(AHandballBall* Ball)
{
	if (!IsValid(Ball) || Ball->GetWorld() != GetWorld() || GetWorld()->GetNetMode() == NM_Client ||
		!Ball->Physics.IsValid())
	{
		return false;
	}
	for (const auto& Entry : Balls)
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

void UBallSimulationSubsystem::UnregisterBall(AHandballBall* Ball)
{
	Balls.RemoveAll([&](const auto& Entry) { return !Entry.IsValid() || Entry.Get() == Ball; });
}

void UBallSimulationSubsystem::TickSimulation(double Elapsed)
{
	Balls.RemoveAll([](const auto& Entry) { return !Entry.IsValid(); });
	if (Balls.Num() == 0)
	{
		return;
	}
	if (GetEnvironmentRevision() == 0 && Clock.GetStatus() == EBallClockStatus::Running)
	{
		Clock.PauseAuthority();
		UE_LOG(LogHandballBall, Warning,
			   TEXT("Static environment changed; paused. Call RebuildStaticEnvironment at an explicit boundary."));
	}
	Clock.BeginRenderFrame(Elapsed);
	while (Clock.HasStepAvailable())
	{
		bool bReady = true;
		for (const auto& Entry : Balls)
		{
			bReady &= Entry->PrepareStep(*CollisionWorld, Clock.GetFrame());
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
		for (const auto& Entry : Balls)
		{
			Entry->CommitStep();
		}
		Clock.ConsumeCompletedStep();
	}
	for (const auto& Entry : Balls)
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
	for (const auto& Entry : Balls)
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
	for (const auto& Entry : Balls)
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
	return Clock.ResumeAuthority(Clock.GetTimebase() + 1);
}
