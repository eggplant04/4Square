#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Engine/EngineBaseTypes.h"
#include "Ball/BallSimulationClock.h"
#include "Ball/UnrealBallCollisionWorld.h"
#include "BallSimulationSubsystem.generated.h"

class AHandballBall;
class FUnrealBallCollisionWorld;
class UBallSimulationSubsystem;

USTRUCT()
struct FOURSQUAREVR_API FBallSimulationTickFunction : public FTickFunction
{
	GENERATED_BODY()
	UBallSimulationSubsystem* Target = nullptr;
	virtual void ExecuteTick(float DeltaTime, ELevelTick TickType, ENamedThreads::Type Thread, const FGraphEventRef& Completion) override;
	virtual FString DiagnosticMessage() override { return TEXT("Handball fixed simulation"); }
};
template<> struct TStructOpsTypeTraits<FBallSimulationTickFunction> : public TStructOpsTypeTraitsBase2<FBallSimulationTickFunction>
{ enum { WithCopy = false }; };

UCLASS()
class FOURSQUAREVR_API UBallSimulationSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()
public:
	virtual ~UBallSimulationSubsystem() override;
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;
	virtual bool DoesSupportWorldType(EWorldType::Type Type) const override;
	bool RegisterBall(AHandballBall* Ball);
	void UnregisterBall(AHandballBall* Ball);
	void TickSimulation(double ElapsedSeconds);
	UFUNCTION(BlueprintCallable, BlueprintAuthorityOnly, Category="Ball")
	bool RebuildStaticEnvironment();
	uint64 GetSimulationFrame() const { return Clock.GetFrame(); }
	uint32 GetEnvironmentRevision() const;
	double GetFrequency() const { return Frequency; }
	bool IsPaused() const { return Clock.GetStatus() != EBallClockStatus::Running; }

private:
	FBallSimulationTickFunction TickFunction;
	FBallSimulationClock Clock;
	FGuid Epoch;
	TArray<TWeakObjectPtr<AHandballBall>> Balls;
	TUniquePtr<FUnrealBallCollisionWorld> CollisionWorld;
	uint64 NextBallId = 1;
	uint32 EnvironmentRevision = 1;
	double Frequency = 120;
	bool bFaultReported = false;
};
