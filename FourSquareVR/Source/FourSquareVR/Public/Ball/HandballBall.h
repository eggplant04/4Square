#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "Ball/BallHistoryBuffer.h"
#include "Ball/BallSimulator.h"
#include "HandballBall.generated.h"

class USphereComponent;
class UStaticMeshComponent;
class UBallSimulationSubsystem;

/** Stage 3 standalone/server ball. Custom snapshots/prediction are deliberately deferred. */
UCLASS(Blueprintable)
class FOURSQUAREVR_API AHandballBall : public AActor
{
	GENERATED_BODY()
  public:
	AHandballBall();
	virtual ~AHandballBall() override;
	virtual void OnConstruction(const FTransform &Transform) override;
	virtual void BeginPlay() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;
	virtual void ApplyWorldOffset(const FVector &Offset, bool bWorldShift) override;

	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ball")
	FBallPhysicsConfig Physics;
	UPROPERTY(EditAnywhere, BlueprintReadOnly, Category = "Ball")
	FBallNetworkSettings HistorySettings;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Test")
	FVector InitialLinearVelocity = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Test")
	FVector InitialAngularVelocity = FVector::ZeroVector;
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Debug")
	bool bDebugDraw = false;
	/** Runtime binding of existing unique LeftGrip/RightGrip sources; no Blueprint asset edits. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Hands")
	bool bAutoRegisterVRHands = true;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ball")
	TObjectPtr<USphereComponent> CollisionProxy;
	UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Ball")
	TObjectPtr<UStaticMeshComponent> VisualMesh;

	UFUNCTION(BlueprintCallable, BlueprintAuthorityOnly, CallInEditor, Category = "Ball|Test")
	bool ResetBall();
	UFUNCTION(BlueprintCallable, BlueprintAuthorityOnly, CallInEditor, Category = "Ball|Test")
	bool LaunchConfiguredBall();
	UFUNCTION(BlueprintCallable, BlueprintAuthorityOnly, Category = "Ball")
	bool LaunchBall(FVector LinearVelocity, FVector AngularVelocity);
	UFUNCTION(BlueprintCallable, BlueprintAuthorityOnly, Category = "Ball")
	bool SetBallState(FVector Position, FVector LinearVelocity, FRotator Rotation, FVector AngularVelocity);
	UFUNCTION(BlueprintCallable, BlueprintAuthorityOnly, Category = "Ball")
	bool SetBallPhysicsConfig(const FBallPhysicsConfig &Config);
	UFUNCTION(BlueprintPure, Category = "Ball")
	FVector GetBallPosition() const
	{
		return State.Position;
	}
	UFUNCTION(BlueprintPure, Category = "Ball")
	FVector GetBallLinearVelocity() const
	{
		return State.LinearVelocity;
	}
	UFUNCTION(BlueprintPure, Category = "Ball")
	FVector GetBallAngularVelocity() const
	{
		return State.AngularVelocity;
	}
	UFUNCTION(BlueprintPure, Category = "Ball")
	bool IsBallResting() const
	{
		return State.Activity == EBallActivityState::Resting;
	}
	UFUNCTION(BlueprintPure, Category = "Ball|Debug")
	UBallSimulationSubsystem *GetSimulationCoordinator() const;
	const FBallState &GetSimulationState() const
	{
		return State;
	}
	const FBallHistoryBuffer &GetHistory() const
	{
		return History;
	}
	const FBallStepOutput &GetLastStepOutput() const
	{
		return *LastOutputStorage;
	}
	bool IsSimulationReady() const
	{
		return bInitialized;
	}

  private:
	friend class UBallSimulationSubsystem;
	UPROPERTY()
	TObjectPtr<USceneComponent> SceneRoot;
	FBallState State, PreparedState;
	FBallPhysicsConfig RuntimePhysics;
	FBallHistoryBuffer History;
	FBallSimulator Simulator;
	TUniquePtr<FBallStepOutput> PreparedOutput, LastOutputStorage;
	FBallStepInputs PreparedInputs;
	FTransform InitialTransform;
	bool bInitialized = false;
	bool InitializeSimulation(FBallIdentity Identity, uint64 Frame, uint32 Environment);
	bool PrepareStep(const IBallCollisionWorld &World, uint64 Frame, uint32 MotionTimebase = 0,
					 uint32 MotionRevision = 0);
	void CommitStep();
	void ProjectState();
	void DrawDiagnostics() const;
};
