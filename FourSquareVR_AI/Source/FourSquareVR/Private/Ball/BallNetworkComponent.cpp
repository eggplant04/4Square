#include "Ball/BallNetworkComponent.h"
#include "Ball/HandballBall.h"
#include "Ball/BallSimulationSubsystem.h"
#include "Ball/VRHandCollisionComponent.h"
#include "Engine/World.h"
#include "EngineUtils.h"
#include "GameFramework/Pawn.h"
#include "HAL/PlatformTime.h"
#include "Net/UnrealNetwork.h"
#include "Ball/BallStateCodec.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Misc/Crc.h"

namespace BallNetworkAudit
{
void Snapshot(const TCHAR *Role, const FBallNetworkSnapshot &S)
{
#if !UE_BUILD_SHIPPING
	static const bool Enabled = FParse::Param(FCommandLine::Get(), TEXT("BallNetworkIPTest"));
	if (Enabled)
	{
		TArray<uint8> Bytes;
		FBallStateCodec::Save(S.State, Bytes);
		UE_LOG(LogTemp, Display, TEXT("NETSNAP Role=%s Seq=%u Frame=%llu Generation=%u Timebase=%u Config=%u CRC=%u"),
			   Role, S.Sequence, S.State.SimulationFrame, S.State.Identity.Generation, S.Timebase,
			   S.State.ConfigVersion, FCrc::MemCrc32(Bytes.GetData(), Bytes.Num()));
	}
#endif
}
} // namespace BallNetworkAudit
UBallNetworkComponent::UBallNetworkComponent()
{
	SetIsReplicatedByDefault(true);
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.TickGroup = TG_PostUpdateWork;
}
void UBallNetworkComponent::BeginPlay()
{
	Super::BeginPlay();
	if (GetNetMode() == NM_Client)
		ReceivedBaselines.SetNum(16);
}
void UBallNetworkComponent::GetLifetimeReplicatedProps(TArray<FLifetimeProperty> &OutLifetimeProps) const
{
	Super::GetLifetimeReplicatedProps(OutLifetimeProps);
	DOREPLIFETIME(UBallNetworkComponent, SnapshotPacket);
}
void UBallNetworkComponent::Publish(bool Force)
{
	auto *Ball = Cast<AHandballBall>(GetOwner());
	if (!Ball || GetNetMode() == NM_Client || !Ball->HasAuthority() || !Ball->IsSimulationReady() ||
		!Transport.IsValid())
		return;
	auto *System = Ball->GetSimulationCoordinator();
	if (!System)
		return;
	const double Now = FPlatformTime::Seconds();
	const bool Paused = System->IsPaused();
	const bool Boundary = !View.bReady || View.Latest.State.Identity != Ball->GetSimulationState().Identity ||
						  View.Latest.Timebase != System->GetTrackingTimebase() || Paused != WasPaused;
	if (!Force && !Boundary && Now < NextPublish)
		return;
	FBallNetworkSnapshot S;
	S.State = Ball->GetSimulationState();
	S.Physics = Ball->GetRuntimePhysics();
	S.History = Ball->HistorySettings;
	S.Sequence = ++Sequence;
	if (!S.Sequence)
		S.Sequence = ++Sequence;
	S.Timebase = System->GetTrackingTimebase();
	S.ServerSeconds = Now;
	S.TimelineSeconds = System->MapMonotonicToTimeline(Now);
	S.bPaused = Paused;
	const double Begin = FPlatformTime::Seconds();
	if (!FBallNetworkCodec::WriteSnapshot(S, SnapshotPacket))
	{
		++Rejected;
		return;
	}
	EncodeMicroseconds = (FPlatformTime::Seconds() - Begin) * 1.e6;
	View.Latest = S;
	View.bReady = true;
	++Published;
	PayloadBytes += SnapshotPacket.Bytes.Num();
	BallNetworkAudit::Snapshot(TEXT("Server"), S);
	if (NextPublish <= 0 || Force || Boundary)
		NextPublish = Now;
	NextPublish += (FMath::FloorToDouble(FMath::Max(0.0, Now - NextPublish) * Transport.SnapshotRate) + 1) /
				   Transport.SnapshotRate;
	WasPaused = Paused;
	Ball->ForceNetUpdate();
}
void UBallNetworkComponent::OnRep_Snapshot()
{
	if (GetNetMode() != NM_Client)
		return;
	FBallNetworkSnapshot S;
	if (!FBallNetworkCodec::ReadSnapshot(SnapshotPacket, S) || !View.Accept(S))
	{
		++Rejected;
		return;
	}
	auto *Ball = Cast<AHandballBall>(GetOwner());
	if (!Ball || !Ball->ApplyAuthoritativeSnapshot(S))
	{
		++Rejected;
		return;
	}
	++Received;
	PayloadBytes += SnapshotPacket.Bytes.Num();
	LastReceive = FPlatformTime::Seconds();
	BallNetworkAudit::Snapshot(TEXT("Client"), S);
	if (ReceivedBaselines.Num())
	{
		ReceivedBaselines[BaselineHead] = S;
		BaselineHead = (BaselineHead + 1) % ReceivedBaselines.Num();
	}
}
void UBallNetworkComponent::TickComponent(float Delta, ELevelTick Type, FActorComponentTickFunction *Tick)
{
	Super::TickComponent(Delta, Type, Tick);
	if (GetNetMode() != NM_Client)
		Publish();
	// Baseline recovery is requested through an owned Pawn, never an unowned ball RPC.
	else if (FPlatformTime::Seconds() - LastReceive > 2)
	{
		for (TActorIterator<APawn> It(GetWorld()); It; ++It)
			if (It->IsLocallyControlled())
				if (auto *Hands = It->FindComponentByClass<UVRHandCollisionComponent>())
				{
					Hands->RequestAuthoritativeBaseline();
					break;
				}
	}
}
FString UBallNetworkComponent::GetNetworkDiagnostics() const
{
	return FString::Printf(TEXT("Published=%llu Received=%llu Rejected=%llu PayloadBytes=%llu EncodeUs=%.3f Seq=%u "
								"Ready=%d BaselineBytes=%llu"),
						   Published, Received, Rejected, PayloadBytes, EncodeMicroseconds,
						   View.bReady ? View.Latest.Sequence : 0, View.bReady,
						   uint64(ReceivedBaselines.GetAllocatedSize()));
}
