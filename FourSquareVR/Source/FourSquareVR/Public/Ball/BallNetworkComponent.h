#pragma once
#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "Ball/BallNetworkTypes.h"
#include "BallNetworkComponent.generated.h"

/** Ball-owned authoritative snapshots. All client-to-server requests use the owned Pawn relay instead. */
UCLASS(ClassGroup = (Ball), meta = (BlueprintSpawnableComponent))
class FOURSQUAREVR_API UBallNetworkComponent : public UActorComponent
{
	GENERATED_BODY()
  public:
	UBallNetworkComponent();
	virtual void BeginPlay() override;
	virtual void TickComponent(float Delta, ELevelTick Type, FActorComponentTickFunction *Tick) override;
	virtual void GetLifetimeReplicatedProps(TArray<FLifetimeProperty> &Out) const override;
	void Publish(bool Force = false);
	const FBallAuthoritativeView &GetView() const
	{
		return View;
	}
	uint64 GetPublishedCount() const
	{
		return Published;
	}
	uint64 GetReceivedCount() const
	{
		return Received;
	}
	uint64 GetPayloadBytes() const
	{
		return PayloadBytes;
	}
	double GetLastReceiveSeconds() const
	{
		return LastReceive;
	}
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Ball|Network") FBallTransportSettings Transport;
	UFUNCTION(BlueprintPure, Category = "Ball|Network") FString GetNetworkDiagnostics() const;

  private:
	UPROPERTY(ReplicatedUsing = OnRep_Snapshot) FBallWirePacket SnapshotPacket;
	UFUNCTION() void OnRep_Snapshot();
	FBallAuthoritativeView View;
	TArray<FBallNetworkSnapshot> ReceivedBaselines;
	int32 BaselineHead = 0;
	uint32 Sequence = 0;
	uint64 Published = 0, Received = 0, Rejected = 0, PayloadBytes = 0;
	double NextPublish = 0, LastReceive = 0, EncodeMicroseconds = 0;
	bool WasPaused = false;
};
