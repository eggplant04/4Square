#pragma once

#include "CoreMinimal.h"
#include "Subsystems/WorldSubsystem.h"
#include "Engine/EngineBaseTypes.h"
#include "Ball/BallSimulationClock.h"
#include "Ball/UnrealBallCollisionWorld.h"
#include "Ball/BallMotionHistory.h"
#include "BallSimulationSubsystem.generated.h"

class AHandballBall;
class FUnrealBallCollisionWorld;
class UBallSimulationSubsystem;
class UVRHandCollisionComponent;
class UBallKinematicColliderComponent;

USTRUCT()
struct FOURSQUAREVR_API FBallSimulationTickFunction : public FTickFunction
{
	GENERATED_BODY()
	UBallSimulationSubsystem *Target = nullptr;
	virtual void ExecuteTick(float DeltaTime, ELevelTick TickType, ENamedThreads::Type Thread,
							 const FGraphEventRef &Completion) override;
	virtual FString DiagnosticMessage() override
	{
		return TEXT("Handball fixed simulation");
	}
};
template <>
struct TStructOpsTypeTraits<FBallSimulationTickFunction> : public TStructOpsTypeTraitsBase2<FBallSimulationTickFunction>
{
	enum
	{
		WithCopy = false
	};
};

UCLASS()
class FOURSQUAREVR_API UBallSimulationSubsystem : public UWorldSubsystem
{
	GENERATED_BODY()
  public:
	virtual ~UBallSimulationSubsystem() override;
	virtual void Initialize(FSubsystemCollectionBase &Collection) override;
	virtual void Deinitialize() override;
	virtual bool DoesSupportWorldType(EWorldType::Type Type) const override;
	bool RegisterBall(AHandballBall *Ball);
	void UnregisterBall(AHandballBall *Ball);
	void TickSimulation(double ElapsedSeconds);
	UFUNCTION(BlueprintCallable, BlueprintAuthorityOnly, Category = "Ball")
	bool RebuildStaticEnvironment();
	uint64 GetSimulationFrame() const
	{
		return Clock.GetFrame();
	}
	uint32 GetEnvironmentRevision() const;
	double GetFrequency() const
	{
		return Frequency;
	}
	UFUNCTION(BlueprintPure, Category = "Ball|Debug")
	bool IsPaused() const
	{
		return Clock.GetStatus() != EBallClockStatus::Running;
	}
	UFUNCTION(BlueprintPure, Category = "Ball|Debug")
	FString GetSimulationDiagnostics() const;
	uint32 RegisterHandSource(UVRHandCollisionComponent *Source);
	void UnregisterHandSource(UVRHandCollisionComponent *Source);
	uint32 RegisterKinematicSource(UBallKinematicColliderComponent *Source);
	void UnregisterKinematicSource(UBallKinematicColliderComponent *Source);
	double GetTrackingTimelineSeconds() const;
	uint32 GetTrackingTimebase() const
	{
		return Clock.GetTimebase();
	}
	const FBallMotionHistory &GetMotionHistory() const
	{
		return MotionHistory;
	}

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
	bool bWaitingForStartup = false;
	int32 StableStartupFrames = 0;
	TArray<TWeakObjectPtr<UVRHandCollisionComponent>> HandSources;
	TArray<TWeakObjectPtr<UBallKinematicColliderComponent>> KinematicSources;
	FBallMotionHistory MotionHistory;
	uint32 NextParticipantId = 1, NextKinematicId = 1;
	double TrackingTimeOrigin = 0, LastMotionBuildMicroseconds = 0, PeakMotionBuildMicroseconds = 0;
	FIntVector LastWorldOrigin = FIntVector::ZeroValue;
	void DiscoverLocalHands();
	void ResetMotionTimebase();
};
