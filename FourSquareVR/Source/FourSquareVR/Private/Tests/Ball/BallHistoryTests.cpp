#include "Misc/AutomationTest.h"

#if WITH_DEV_AUTOMATION_TESTS

#include "BallTestFixtures.h"
#include "Ball/BallReplayRunner.h"
#include "Ball/RecordedBallCollisionWorld.h"
#include "Misc/Crc.h"
#include <limits>

namespace
{
	constexpr EAutomationTestFlags HistoryFlags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;
	bool RecordTrace(FBallHistoryBuffer& History, FBallState& State, int32 Count,
		const FBallPhysicsConfig& Config, const IBallCollisionWorld& World, FBallState* StateAt510 = nullptr)
	{
		FBallSimulator Simulator;
		FBallStepOutput Output;
		for (int32 I = 0; I < Count; ++I)
		{
			FBallStepInputs Inputs = BallTests::Inputs(State);
			Inputs.ForceCount = 1;
			Inputs.Forces[0] = {1, 0, FVector(2 * FMath::Sin(State.SimulationFrame * 0.01), 0, 0), FVector::ZeroVector};
			if (History.Push({State, Inputs}) != EBallHistoryWrite::Stored
				|| Simulator.SimulateStep(State, Inputs, Config, World, Output) != EBallStepResult::Success) { return false; }
			if (StateAt510 && State.SimulationFrame == 510) { *StateAt510 = State; }
		}
		return true;
	}
	void UpdateCRC(TArray<uint8>& Bytes)
	{
		const uint32 CRC = FCrc::MemCrc32(Bytes.GetData() + 14, Bytes.Num() - 14);
		FMemory::Memcpy(Bytes.GetData() + 10, &CRC, sizeof(CRC)); // Format 1 header, little-endian Win64 fixture.
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallCheckpointTest, "FourSquareVR.Ball.History.CompleteCheckpointRoundTrip", HistoryFlags)
bool FBallCheckpointTest::RunTest(const FString& Parameters)
{
	const FBallState Source = BallTests::RichState();
	TestTrue(TEXT("Rich checkpoint valid"), Source.IsValid());
	TArray<uint8> Bytes;
	TestTrue(TEXT("All live fields saved"), FBallStateCodec::Save(Source, Bytes));
	FBallState Restored = BallTests::State();
	TestTrue(TEXT("Load valid checkpoint"), FBallStateCodec::Load(Bytes, Restored));
	TestTrue(TEXT("Every active field re-encodes identically"), BallTests::ExactlyEqual(Source, Restored));
	TestEqual(TEXT("Contact episodes retained"), Restored.Contacts[7].Episode, uint32(8));
	TestEqual(TEXT("Force counter retained"), Restored.ForceStates[3].Counter, uint64(53));
	TestEqual(TEXT("Sleep retained"), Restored.SleepDwellTime, 0.375);
	TestTrue(TEXT("Rest/support flags retained"), Restored.Activity == EBallActivityState::Resting && Restored.bHasSupport);
	AddInfo(FString::Printf(TEXT("CheckpointBytes=%d StateBytes=%llu HistoryFrameBytes=%llu"), Bytes.Num(),
		static_cast<uint64>(sizeof(FBallState)), static_cast<uint64>(sizeof(FBallHistoryFrame))));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallCodecCorruptionTest, "FourSquareVR.Ball.History.CheckpointCorruptionAndBounds", HistoryFlags)
bool FBallCodecCorruptionTest::RunTest(const FString& Parameters)
{
	TArray<uint8> Bytes;
	FBallStateCodec::Save(BallTests::RichState(), Bytes);
	FBallState Destination = BallTests::State(444);
	const FBallState Before = Destination;
	for (int32 Length = 0; Length < Bytes.Num(); ++Length)
	{
		if (FBallStateCodec::Load(MakeArrayView(Bytes.GetData(), Length), Destination))
		{ AddError(TEXT("Truncated checkpoint accepted")); return false; }
	}
	TArray<uint8> Bad = Bytes;
	Bad[4] = 99;
	TestFalse(TEXT("Unknown schema rejected"), FBallStateCodec::Load(Bad, Destination));
	Bad = Bytes;
	Bad.Last() ^= 1;
	TestFalse(TEXT("Payload corruption rejected"), FBallStateCodec::Load(Bad, Destination));
	Bad = Bytes;
	Bad.Add(0);
	TestFalse(TEXT("Trailing bytes rejected"), FBallStateCodec::Load(Bad, Destination));
	Bad = Bytes;
	const double NaN = std::numeric_limits<double>::quiet_NaN();
	FMemory::Memcpy(Bad.GetData() + 14 + 48, &NaN, sizeof(NaN));
	UpdateCRC(Bad);
	TestFalse(TEXT("NaN rejected even with valid CRC"), FBallStateCodec::Load(Bad, Destination));
	Bad = Bytes;
	const int32 ExcessiveCount = MAX_int32;
	FMemory::Memcpy(Bad.GetData() + 14 + 234, &ExcessiveCount, sizeof(ExcessiveCount));
	UpdateCRC(Bad);
	TestFalse(TEXT("Malformed count rejected before array traversal"), FBallStateCodec::Load(Bad, Destination));
	Bad.SetNumZeroed(FBallStateCodec::MaximumEncodedBytes + 1);
	TestFalse(TEXT("Encoded size bound"), FBallStateCodec::Load(Bad, Destination));
	TestTrue(TEXT("All failed loads preserve whole destination"), BallTests::ExactlyEqual(Destination, Before));
	FBallState Invalid = BallTests::RichState();
	Invalid.ContactCount = BallLimits::Contacts + 1;
	Bad = {1, 2, 3};
	TestFalse(TEXT("Invalid save rejected"), FBallStateCodec::Save(Invalid, Bad));
	TestTrue(TEXT("Failed save preserves destination"), Bad == TArray<uint8>({1, 2, 3}));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallHistoryWrapTest, "FourSquareVR.Ball.History.RingWrapAndInvalidFrames", HistoryFlags)
bool FBallHistoryWrapTest::RunTest(const FString& Parameters)
{
	FBallHistoryBuffer History;
	FBallState State = BallTests::State(10);
	TestTrue(TEXT("Preallocate ring"), History.Initialize(4, State.Identity));
	const SIZE_T Bytes = History.GetAllocatedBytes();
	for (uint64 Frame = 10; Frame < 1010; ++Frame)
	{
		State.SimulationFrame = Frame;
		State.Position.X = Frame;
		if (History.Push({State, BallTests::Inputs(State)}) != EBallHistoryWrite::Stored)
		{ AddError(TEXT("Contiguous ring write failed")); return false; }
	}
	TestEqual(TEXT("Bounded occupancy"), History.Num(), 4);
	TestEqual(TEXT("No ring allocation growth"), History.GetAllocatedBytes(), Bytes);
	TestEqual(TEXT("Oldest retained"), History.GetOldestFrame(), uint64(1006));
	FBallHistoryFrame Record;
	TestFalse(TEXT("Wrapped-out frame unavailable"), History.CopyFrame(1005, State.Identity, Record));
	TestFalse(TEXT("Future frame unavailable"), History.CopyFrame(1010, State.Identity, Record));
	TestTrue(TEXT("Exact slot identity checked"), History.CopyFrame(1007, State.Identity, Record));
	TestEqual(TEXT("Retained content not aliased"), Record.State.Position.X, 1007.0);
	Record.State.Position.X = -100;
	History.CopyFrame(1007, State.Identity, Record);
	TestEqual(TEXT("Caller receives owned copy"), Record.State.Position.X, 1007.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallPinnedHistoryTest, "FourSquareVR.Ball.History.PinnedCapacityAndGeneration", HistoryFlags)
bool FBallPinnedHistoryTest::RunTest(const FString& Parameters)
{
	FBallHistoryBuffer History;
	FBallState State = BallTests::State(10);
	History.Initialize(4, State.Identity);
	for (uint64 Frame = 10; Frame < 14; ++Frame)
	{ State.SimulationFrame = Frame; History.Push({State, BallTests::Inputs(State)}); }
	TestTrue(TEXT("Replay inputs pinned"), History.PinFromFrame(11));
	State.SimulationFrame = 14;
	TestTrue(TEXT("Unpinned oldest may retire"), History.Push({State, BallTests::Inputs(State)}) == EBallHistoryWrite::Stored);
	State.SimulationFrame = 15;
	TestTrue(TEXT("Cannot overwrite pinned history"), History.Push({State, BallTests::Inputs(State)}) == EBallHistoryWrite::PinnedCapacity);
	TestEqual(TEXT("No partial ring advance"), History.GetNewestFrame(), uint64(14));
	History.Unpin();
	TestTrue(TEXT("Explicit release permits progress"), History.Push({State, BallTests::Inputs(State)}) == EBallHistoryWrite::Stored);
	TestTrue(TEXT("Duplicate boundary not silently replaced"), History.Push({State, BallTests::Inputs(State)}) == EBallHistoryWrite::NonContiguous);
	State.SimulationFrame = 17;
	TestTrue(TEXT("Missing boundary rejected"), History.Push({State, BallTests::Inputs(State)}) == EBallHistoryWrite::NonContiguous);
	const FBallIdentity Old = State.Identity;
	++State.Identity.Generation;
	State.SimulationFrame = 16;
	TestTrue(TEXT("Wrong generation write rejected"), History.Push({State, BallTests::Inputs(State)}) == EBallHistoryWrite::WrongIdentity);
	TestTrue(TEXT("Explicit reset changes generation"), History.Reset(State.Identity));
	History.Push({State, BallTests::Inputs(State)});
	FBallHistoryFrame Record;
	TestFalse(TEXT("Old generation cannot read new slot"), History.CopyFrame(16, Old, Record));
	TestTrue(TEXT("New generation reads its slot"), History.CopyFrame(16, State.Identity, Record));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallRecordedReplayTest, "FourSquareVR.Ball.History.RecordedReplayAndRestoreContinue", HistoryFlags)
bool FBallRecordedReplayTest::RunTest(const FString& Parameters)
{
	FBallPhysicsConfig Config;
	FRecordedBallCollisionWorld World;
	FBallState State = BallTests::State();
	State.AngularVelocity = FVector(0.3, 0.7, 0.1);
	State.DeterministicSeed = 99;
	State.ForceStateCount = 1;
	State.ForceStates[0] = {1, 2, 53, {1, 2, 3, 4}};
	FBallHistoryBuffer History;
	History.Initialize(256, State.Identity);
	TestTrue(TEXT("Recorded trace created"), RecordTrace(History, State, 120, Config, World));
	FBallHistoryFrame Checkpoint;
	History.CopyFrame(90, State.Identity, Checkpoint);
	TArray<uint8> Bytes;
	FBallStateCodec::Save(Checkpoint.State, Bytes);
	FBallState Restored;
	TestTrue(TEXT("Full historical state restored"), FBallStateCodec::Load(Bytes, Restored));
	FBallSimulator Simulator;
	FBallStepOutput Output;
	for (uint64 Frame = 90; Frame < 120; ++Frame)
	{
		History.CopyFrame(Frame, State.Identity, Checkpoint);
		if (Simulator.SimulateStep(Restored, Checkpoint.Inputs, Config, World, Output) != EBallStepResult::Success)
		{ AddError(TEXT("Restored continuation failed")); return false; }
	}
	TestTrue(TEXT("Serialized restore + saved inputs reproduce complete trace"), BallTests::ExactlyEqual(State, Restored));
	FBallReplayRunner Runner;
	FBallReplayReport Report;
	FBallState Replay = BallTests::State(999);
	TestTrue(TEXT("Bounded replay succeeds"), Runner.Replay(History, State.Identity, 90, 120, Config, {}, World, Replay, Report) == EBallReplayResult::Success);
	TestEqual(TEXT("Exact replay frame count"), Report.ExecutedFrames, uint32(30));
	TestEqual(TEXT("Replay query work accounted"), Report.Queries, uint32(30));
	TestEqual(TEXT("Replay microinterval work accounted"), Report.Microintervals, uint32(30));
	TestTrue(TEXT("Replay whole state equivalent"), BallTests::ExactlyEqual(State, Replay));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallReplayFailureTest, "FourSquareVR.Ball.History.ReplayBudgetsAndAtomicFailure", HistoryFlags)
bool FBallReplayFailureTest::RunTest(const FString& Parameters)
{
	FBallPhysicsConfig Config;
	FRecordedBallCollisionWorld World;
	FBallState State = BallTests::State();
	FBallHistoryBuffer History;
	History.Initialize(256, State.Identity);
	TestTrue(TEXT("Fixture trace"), RecordTrace(History, State, 120, Config, World));
	FBallReplayRunner Runner;
	FBallReplayReport Report;
	FBallState Destination = BallTests::RichState();
	const FBallState Before = Destination;
	TestTrue(TEXT("Budget rejected before simulation"), Runner.Replay(History, State.Identity, 0, 120, Config, {}, World, Destination, Report) == EBallReplayResult::BudgetExceeded);
	TestEqual(TEXT("Zero work on budget failure"), Report.ExecutedFrames, uint32(0));
	TestTrue(TEXT("Reverse range rejected"), Runner.Replay(History, State.Identity, 120, 90, Config, {}, World, Destination, Report) == EBallReplayResult::InvalidRange);
	TestTrue(TEXT("Missing inputs rejected"), Runner.Replay(History, State.Identity, 110, 130, Config, {}, World, Destination, Report) == EBallReplayResult::MissingFrame);
	FBallIdentity Wrong = State.Identity;
	++Wrong.Generation;
	TestTrue(TEXT("Old/reset identity rejected"), Runner.Replay(History, Wrong, 90, 120, Config, {}, World, Destination, Report) == EBallReplayResult::MissingFrame);
	FRecordedBallCollisionWorld ChangedWorld(2);
	TestTrue(TEXT("Historical environment mismatch rejected"), Runner.Replay(History, State.Identity, 90, 120, Config, {}, ChangedWorld, Destination, Report) == EBallReplayResult::IncompatibleConfiguration);
	// Same versioned fixture with a blocking plane forces a mid-replay transactional failure.
	FRecordedBallCollisionWorld Blocking;
	Blocking.AddPlane({1, 1}, FVector(0, 0, -300), FVector::UpVector);
	TestTrue(TEXT("Unsupported collision fails replay"), Runner.Replay(History, State.Identity, 90, 120, Config, {}, Blocking, Destination, Report) == EBallReplayResult::SimulationFailed);
	TestTrue(TEXT("No failed replay ever modified destination"), BallTests::ExactlyEqual(Destination, Before));
	TestEqual(TEXT("Source history not rewritten"), History.Num(), 120);
	TestEqual(TEXT("Every failed request instrumented"), Runner.GetPerformanceCounters().FailedReplays, uint64(6));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallReplayPerformanceTest, "FourSquareVR.Ball.History.BoundedReplayPerformance", HistoryFlags)
bool FBallReplayPerformanceTest::RunTest(const FString& Parameters)
{
	FBallPhysicsConfig Config;
	FBallNetworkSettings Settings;
	FRecordedBallCollisionWorld World;
	FBallState State = BallTests::State(), Reference;
	FBallHistoryBuffer History;
	History.Initialize(Settings.HistoryCapacity, State.Identity);
	TestTrue(TEXT("Trace wraps ring before measurement"), RecordTrace(History, State, 512, Config, World, &Reference));
	const SIZE_T Allocated = History.GetAllocatedBytes();
	FBallReplayRunner Runner;
	FBallReplayReport Report;
	FBallState Replay;
	TArray<double> Durations;
	Durations.Reserve(512); // Test reporting storage, outside the measured replay operation.
	for (int32 I = 0; I < 512; ++I)
	{
		if (Runner.Replay(History, State.Identity, 480, 510, Config, Settings, World, Replay, Report) != EBallReplayResult::Success)
		{ AddError(TEXT("Benchmark replay failed")); return false; }
		Durations.Add(Report.DurationMicroseconds);
	}
	Durations.Sort();
	TestTrue(TEXT("Benchmark still verifies replay result"), BallTests::ExactlyEqual(Reference, Replay));
	TestEqual(TEXT("No history storage growth"), History.GetAllocatedBytes(), Allocated);
	TestEqual(TEXT("All replay work bounded and counted"), Runner.GetPerformanceCounters().ReplayedFrames, uint64(512 * 30));
	TestEqual(TEXT("No failed benchmark requests"), Runner.GetPerformanceCounters().FailedReplays, uint64(0));
	TestTrue(TEXT("Finite measured total"), FMath::IsFinite(Runner.GetPerformanceCounters().TotalReplayMicroseconds));
	AddInfo(FString::Printf(TEXT("PC_FREE_FLIGHT_ONLY Iterations=512 FramesPerReplay=30 HistoryCapacity=%d HistoryBytes=%llu StateBytes=%llu FrameBytes=%llu MeanReplayUs=%.3f MedianUs=%.3f P95Us=%.3f MaxUs=%.3f; excludes scene queries, hand CCD, contacts, networking; NOT Quest qualification"),
		History.Capacity(), static_cast<uint64>(Allocated), static_cast<uint64>(sizeof(FBallState)), static_cast<uint64>(sizeof(FBallHistoryFrame)),
		Runner.GetPerformanceCounters().TotalReplayMicroseconds / 512, Durations[256], Durations[486], Durations.Last()));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallZeroLengthReplayTest, "FourSquareVR.Ball.History.ZeroLengthReplayValidation", HistoryFlags)
bool FBallZeroLengthReplayTest::RunTest(const FString& Parameters)
{
	FBallHistoryBuffer History;
	FBallState Source = BallTests::State(9);
	History.Initialize(32, Source.Identity);
	History.Push({Source, BallTests::Inputs(Source)});
	FBallReplayRunner Runner;
	FBallReplayReport Report;
	FBallState Destination = BallTests::RichState();
	FBallPhysicsConfig Config;
	FRecordedBallCollisionWorld World;
	TestTrue(TEXT("Zero-step restoration valid"), Runner.Replay(History, Source.Identity, 9, 9, Config, {}, World, Destination, Report) == EBallReplayResult::Success);
	TestTrue(TEXT("Full boundary copied without numerical work"), BallTests::ExactlyEqual(Source, Destination));
	TestEqual(TEXT("No replay steps"), Report.ExecutedFrames, uint32(0));
	Config.Version = 2;
	TestTrue(TEXT("Even zero-step restore validates config"), Runner.Replay(History, Source.Identity, 9, 9, Config, {}, World, Destination, Report) == EBallReplayResult::IncompatibleConfiguration);
	TestTrue(TEXT("Invalid zero-step restore leaves destination"), BallTests::ExactlyEqual(Source, Destination));
	return true;
}

#endif
