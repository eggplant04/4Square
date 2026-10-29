#include "Ball/BallStateCodec.h"
#include "Misc/Crc.h"
#include "Serialization/MemoryReader.h"
#include "Serialization/MemoryWriter.h"

namespace
{
	constexpr uint32 Magic = 0x324C4142; // BAL2
	constexpr int32 HeaderBytes = 14;

	void Vector(FArchive& Ar, FVector& Value) { Ar << Value.X << Value.Y << Value.Z; }
	void Collider(FArchive& Ar, FBallColliderId& Id) { Ar << Id.Value << Id.Generation; }
	bool Flag(FArchive& Ar, bool& Value)
	{
		uint8 Encoded = Value ? 1 : 0;
		Ar << Encoded;
		if (Encoded > 1) { return false; }
		if (Ar.IsLoading()) { Value = Encoded != 0; }
		return !Ar.IsError();
	}
	bool Fields(FArchive& Ar, FBallState& State)
	{
		Ar << State.Identity.Epoch << State.Identity.BallId << State.Identity.Generation << State.SimulationFrame;
		Ar << State.ConfigVersion << State.MaterialVersion << State.EnvironmentRevision;
		Vector(Ar, State.Position);
		Vector(Ar, State.LinearVelocity);
		Ar << State.Rotation.X << State.Rotation.Y << State.Rotation.Z << State.Rotation.W;
		Vector(Ar, State.AngularVelocity);
		uint8 Activity = static_cast<uint8>(State.Activity);
		Ar << Activity << State.SleepDwellTime;
		if (Activity > static_cast<uint8>(EBallActivityState::Resting) || !Flag(Ar, State.bHasSupport)) { return false; }
		State.Activity = static_cast<EBallActivityState>(Activity);
		Collider(Ar, State.SupportCollider);
		Vector(Ar, State.SupportNormal);
		Vector(Ar, State.SupportVelocity);
		Ar << State.DeterministicSeed << State.NextEventOrdinal << State.ContactCount;
		if (State.ContactCount < 0 || State.ContactCount > BallLimits::Contacts || Ar.IsError()) { return false; }
		for (int32 I = 0; I < State.ContactCount; ++I)
		{
			FBallContactState& Contact = State.Contacts[I];
			Collider(Ar, Contact.Collider);
			Ar << Contact.Episode << Contact.LastImpactFrame;
			Vector(Ar, Contact.Normal);
			Vector(Ar, Contact.ContactPoint);
			Vector(Ar, Contact.AccumulatedTangentImpulse);
			Ar << Contact.AccumulatedNormalImpulse << Contact.Separation;
			if (!Flag(Ar, Contact.bTouching) || !Flag(Ar, Contact.bSupporting) || !Flag(Ar, Contact.bRestitutionApplied)) { return false; }
		}
		Ar << State.ForceStateCount;
		if (State.ForceStateCount < 0 || State.ForceStateCount > BallLimits::ForceProviders || Ar.IsError()) { return false; }
		for (int32 I = 0; I < State.ForceStateCount; ++I)
		{
			FBallForceState& Force = State.ForceStates[I];
			Ar << Force.ProviderId << Force.Version << Force.Counter;
			for (double& Value : Force.Values) { Ar << Value; }
		}
		return !Ar.IsError();
	}
}

bool FBallStateCodec::Save(const FBallState& State, TArray<uint8>& OutBytes)
{
	if (!State.IsValid()) { return false; }
	TArray<uint8> Payload;
	Payload.Reserve(MaximumEncodedBytes);
	FMemoryWriter PayloadWriter(Payload, true);
	PayloadWriter.SetByteSwapping(!PLATFORM_LITTLE_ENDIAN);
	FBallState Copy = State;
	if (!Fields(PayloadWriter, Copy) || Payload.Num() + HeaderBytes > MaximumEncodedBytes) { return false; }
	TArray<uint8> Encoded;
	Encoded.Reserve(HeaderBytes + Payload.Num());
	FMemoryWriter Writer(Encoded, true);
	Writer.SetByteSwapping(!PLATFORM_LITTLE_ENDIAN);
	uint32 FileMagic = Magic;
	uint16 Version = FormatVersion;
	uint32 Size = Payload.Num();
	uint32 CRC = FCrc::MemCrc32(Payload.GetData(), Payload.Num());
	Writer << FileMagic << Version << Size << CRC;
	Writer.Serialize(Payload.GetData(), Payload.Num());
	if (Writer.IsError()) { return false; }
	OutBytes = MoveTemp(Encoded);
	return true;
}

bool FBallStateCodec::Load(TConstArrayView<uint8> Bytes, FBallState& OutState)
{
	if (Bytes.Num() < HeaderBytes || Bytes.Num() > MaximumEncodedBytes) { return false; }
	// Reader storage is bounded before allocation; malformed counts cannot grow state storage.
	TArray<uint8> Storage;
	Storage.Append(Bytes.GetData(), Bytes.Num());
	FMemoryReader Reader(Storage, true);
	Reader.SetByteSwapping(!PLATFORM_LITTLE_ENDIAN);
	uint32 FileMagic = 0, Size = 0, CRC = 0;
	uint16 Version = 0;
	Reader << FileMagic << Version << Size << CRC;
	if (Reader.IsError() || FileMagic != Magic || Version != FormatVersion
		|| Size != static_cast<uint32>(Bytes.Num() - HeaderBytes)
		|| CRC != FCrc::MemCrc32(Bytes.GetData() + HeaderBytes, Size)) { return false; }
	FBallState Candidate;
	if (!Fields(Reader, Candidate) || !Reader.AtEnd() || !Candidate.IsValid()) { return false; }
	OutState = Candidate;
	return true;
}
