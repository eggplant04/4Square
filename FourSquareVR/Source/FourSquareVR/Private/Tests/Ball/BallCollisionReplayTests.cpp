#include "Misc/AutomationTest.h"
#if WITH_DEV_AUTOMATION_TESTS
#include "BallTestFixtures.h"
#include "Ball/BallReplayRunner.h"
#include "Ball/BallSimulationClock.h"
#include "Ball/RecordedBallCollisionWorld.h"
#include "Misc/Crc.h"

namespace
{
constexpr auto ReplayFlags = EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter;
FBallPhysicsConfig CollisionConfig()
{
	FBallPhysicsConfig C;
	C.Gravity = FVector::ZeroVector;
	C.LinearDrag = C.AngularDrag = C.SurfaceFriction = C.RollingResistance = C.SpinFriction = 0;
	C.Restitution = 1;
	return C;
}
void Court(FRecordedBallCollisionWorld& W)
{
	W.AddPlane({1, 1}, FVector(-100, 0, 0), FVector::ForwardVector);
	W.AddPlane({2, 1}, FVector(100, 0, 0), -FVector::ForwardVector);
	W.AddPlane({3, 1}, FVector(0, -100, 0), FVector::RightVector);
	W.AddPlane({4, 1}, FVector(0, 100, 0), -FVector::RightVector);
	W.AddPlane({5, 1}, FVector(0, 0, 0), FVector::UpVector);
	W.AddPlane({6, 1}, FVector(0, 0, 200), -FVector::UpVector);
}
FBallState MovingState()
{
	auto S = BallTests::State();
	S.Position = FVector(0, 0, 100);
	S.LinearVelocity = FVector(12345, 5678, -9876);
	return S;
}
bool SameEvent(const FBallCollisionEvent& A, const FBallCollisionEvent& B)
{
	return A.Ball == B.Ball && A.Collider == B.Collider && A.FeatureId == B.FeatureId && A.Episode == B.Episode &&
		   A.Ordinal == B.Ordinal && A.Time.Frame == B.Time.Frame && A.Time.Fraction == B.Time.Fraction &&
		   A.Category == B.Category && A.Position.Equals(B.Position, 1.e-9) && A.Normal.Equals(B.Normal, 1.e-9) &&
		   A.IncomingVelocity.Equals(B.IncomingVelocity, 1.e-9) &&
		   A.OutgoingVelocity.Equals(B.OutgoingVelocity, 1.e-9) &&
		   A.IncomingAngularVelocity.Equals(B.IncomingAngularVelocity, 1.e-9) &&
		   A.OutgoingAngularVelocity.Equals(B.OutgoingAngularVelocity, 1.e-9) &&
		   FMath::IsNearlyEqual(A.NormalImpulse, B.NormalImpulse, 1.e-9);
}
} // namespace

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallCollisionFPS, "FourSquareVR.Ball.Collision.RenderFramerateEquivalence",
								 ReplayFlags)
bool FBallCollisionFPS::RunTest(const FString&)
{
	FBallState Reference;
	for (int32 FPS : {30, 72, 90, 120, 144, 240})
	{
		FRecordedBallCollisionWorld W;
		Court(W);
		const auto C = CollisionConfig();
		auto S = MovingState();
		FBallSimulationClock Clock;
		Clock.Initialize({}, EBallClockRole::Authority);
		FBallSimulator Sim;
		FBallStepOutput O;
		for (int32 I = 0; I < FPS * 2; ++I)
		{
			Clock.BeginRenderFrame(1.0 / FPS);
			while (Clock.HasStepAvailable())
			{
				if (Sim.SimulateStep(S, BallTests::Inputs(S), C, W, O) != EBallStepResult::Success)
				{
					AddError(TEXT("Collision trace failed"));
					return false;
				}
				Clock.ConsumeCompletedStep();
			}
		}
		TestEqual(TEXT("240 fixed frames at every rendering FPS"), S.SimulationFrame, uint64(240));
		TestTrue(TEXT("Real repeated collision events occurred"), S.NextEventOrdinal > 100);
		if (FPS == 30)
		{
			Reference = S;
		}
		else
		{
			TestTrue(TEXT("Entire collision state independent of render FPS"), BallTests::ExactlyEqual(S, Reference));
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallCollisionReplay,
								 "FourSquareVR.Ball.Collision.CheckpointReplayAndEventEquivalence", ReplayFlags)
bool FBallCollisionReplay::RunTest(const FString&)
{
	FRecordedBallCollisionWorld W;
	Court(W);
	auto C = CollisionConfig();
	auto S = MovingState();
	FBallHistoryBuffer H;
	H.Initialize(256, S.Identity);
	FBallSimulator Sim;
	FBallStepOutput O;
	FBallState Reference;
	TArray<FBallCollisionEvent> Expected;
	for (int32 I = 0; I < 512; ++I)
	{
		auto Inputs = BallTests::Inputs(S);
		H.Push({S, Inputs});
		if (Sim.SimulateStep(S, Inputs, C, W, O) != EBallStepResult::Success)
		{
			AddError(TEXT("Recording collision trace failed"));
			return false;
		}
		if (I >= 480 && I < 510)
		{
			for (int32 J = 0; J < O.EventCount; ++J)
			{
				Expected.Add(O.Events[J]);
			}
		}
		if (I == 509)
		{
			Reference = S;
		}
	}
	TestTrue(TEXT("Collision events in revision window"), Expected.Num() > 10);
	FBallHistoryFrame Record;
	H.CopyFrame(480, S.Identity, Record);
	TArray<uint8> Encoded;
	FBallStateCodec::Save(Record.State, Encoded);
	FBallState Restored;
	TestTrue(TEXT("Complete collision checkpoint restored"), FBallStateCodec::Load(Encoded, Restored));
	int32 Event = 0;
	for (uint64 I = 480; I < 510; ++I)
	{
		H.CopyFrame(I, S.Identity, Record);
		if (Sim.SimulateStep(Restored, Record.Inputs, C, W, O) != EBallStepResult::Success)
		{
			AddError(TEXT("Restored collision step failed"));
			return false;
		}
		for (int32 J = 0; J < O.EventCount; ++J)
		{
			if (Event >= Expected.Num() || !SameEvent(O.Events[J], Expected[Event++]))
			{
				AddError(TEXT("Collision event identity/timing/response changed after restore"));
				return false;
			}
		}
	}
	TestEqual(TEXT("Every collision event regenerated exactly once"), Event, Expected.Num());
	TestTrue(TEXT("All live state matches uninterrupted trace"), BallTests::ExactlyEqual(Restored, Reference));
	FBallReplayRunner Runner;
	FBallReplayReport Report;
	FBallState Replay;
	TestTrue(TEXT("Bounded collision replay"),
			 Runner.Replay(H, S.Identity, 480, 510, C, {}, W, Replay, Report) == EBallReplayResult::Success);
	TestTrue(TEXT("Replay runner preserves complete collision state"), BallTests::ExactlyEqual(Replay, Reference));
	TestEqual(TEXT("Thirty frames"), Report.ExecutedFrames, uint32(30));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallLegacyCheckpoint, "FourSquareVR.Ball.History.LegacyV1FeatureMigration",
								 ReplayFlags)
bool FBallLegacyCheckpoint::RunTest(const FString&)
{
	auto Source = BallTests::RichState();
	TArray<uint8> Bytes;
	FBallStateCodec::Save(Source, Bytes);
	// Explicit v1 fixture layout: 14-byte header, contact records at payload 238; v2 adds four bytes after each
	// collider.
	for (int32 I = Source.ContactCount - 1; I >= 0; --I)
	{
		Bytes.RemoveAt(14 + 238 + I * 119 + 12, 4, EAllowShrinking::No);
		Source.Contacts[I].FeatureId = 0;
	}
	const uint16 Version = 1;
	const uint32 Size = Bytes.Num() - 14;
	const uint32 CRC = FCrc::MemCrc32(Bytes.GetData() + 14, Size);
	FMemory::Memcpy(Bytes.GetData() + 4, &Version, 2);
	FMemory::Memcpy(Bytes.GetData() + 6, &Size, 4);
	FMemory::Memcpy(Bytes.GetData() + 10, &CRC, 4);
	FBallState Restored;
	TestTrue(TEXT("Stage 2 v1 loads"), FBallStateCodec::Load(Bytes, Restored));
	TestTrue(TEXT("All legacy fields retained, feature defaults zero"), BallTests::ExactlyEqual(Restored, Source));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallRepeatedEnergy, "FourSquareVR.Ball.Collision.RepeatedNarrowContactsAndEnergy",
								 ReplayFlags)
bool FBallRepeatedEnergy::RunTest(const FString&)
{
	FRecordedBallCollisionWorld W;
	W.AddPlane({1, 1}, FVector::ZeroVector, FVector::ForwardVector);
	W.AddPlane({2, 1}, FVector(6.2, 0, 0), -FVector::ForwardVector);
	auto C = CollisionConfig();
	auto S = BallTests::State();
	S.Position.X = 3.1;
	S.LinearVelocity.X = 120;
	FBallSimulator Sim;
	FBallStepOutput O;
	for (int32 I = 0; I < 1200; ++I)
	{
		if (Sim.SimulateStep(S, BallTests::Inputs(S), C, W, O) != EBallStepResult::Success)
		{
			AddError(TEXT("Valid narrow channel failed"));
			return false;
		}
		if (S.Position.X < 3 - 1.e-7 || S.Position.X > 3.2 + 1.e-7 ||
			FMath::Abs(FMath::Abs(S.LinearVelocity.X) - 120) > 1.e-7)
		{
			AddError(TEXT("Narrow channel gained energy or penetrated"));
			return false;
		}
	}
	TestTrue(TEXT("Thousands of real contacts, monotonic ordinals"), S.NextEventOrdinal > 5000);
	TestTrue(TEXT("Cache remains bounded"), S.ContactCount <= BallLimits::Contacts);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FBallCollisionPerformance, "FourSquareVR.Ball.Collision.BoundedReplayPerformance",
								 ReplayFlags)
bool FBallCollisionPerformance::RunTest(const FString&)
{
	FRecordedBallCollisionWorld W;
	Court(W);
	auto C = CollisionConfig();
	auto S = MovingState();
	FBallHistoryBuffer H;
	H.Initialize(256, S.Identity);
	FBallState Reference;
	FBallSimulator Sim;
	FBallStepOutput O;
	for (int32 I = 0; I < 512; ++I)
	{
		auto Inputs = BallTests::Inputs(S);
		H.Push({S, Inputs});
		if (Sim.SimulateStep(S, Inputs, C, W, O) != EBallStepResult::Success)
		{
			AddError(TEXT("Benchmark trace failed"));
			return false;
		}
		if (I == 509)
		{
			Reference = S;
		}
	}
	FBallReplayRunner Runner;
	FBallReplayReport Report;
	FBallState Replay;
	TArray<double> Durations;
	Durations.Reserve(512);
	const SIZE_T Bytes = H.GetAllocatedBytes();
	for (int32 I = 0; I < 512; ++I)
	{
		if (Runner.Replay(H, S.Identity, 480, 510, C, {}, W, Replay, Report) != EBallReplayResult::Success)
		{
			AddError(TEXT("Benchmark replay failed"));
			return false;
		}
		Durations.Add(Report.DurationMicroseconds);
	}
	Durations.Sort();
	TestTrue(TEXT("Benchmark also verifies state"), BallTests::ExactlyEqual(Replay, Reference));
	TestEqual(TEXT("Ring memory bounded"), H.GetAllocatedBytes(), Bytes);
	AddInfo(
		FString::Printf(TEXT("PC_ANALYTIC_COLLISION Iterations=512 FramesPerReplay=30 Speed=%.1fcm/s HistoryBytes=%llu "
							 "StateBytes=%llu StepOutputBytes=%llu MedianReplayUs=%.3f P95Us=%.3f MaxUs=%.3f "
							 "QueriesPerReplay=%u; six planes, no engine query cost; NOT Quest qualification"),
						MovingState().LinearVelocity.Size(), static_cast<uint64>(Bytes),
						static_cast<uint64>(sizeof(FBallState)), static_cast<uint64>(sizeof(FBallStepOutput)),
						Durations[256], Durations[486], Durations.Last(), Report.Queries));
	return true;
}
#endif
