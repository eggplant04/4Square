#pragma once

#include "Ball/BallHistoryBuffer.h"
#include "Ball/BallStateCodec.h"

namespace BallTests
{
	inline FBallState State(uint64 Frame = 0)
	{
		FBallState Result;
		Result.Identity = {FGuid(1, 2, 3, 4), 7, 1};
		Result.SimulationFrame = Frame;
		return Result;
	}
	inline FBallStepInputs Inputs(const FBallState& State)
	{
		FBallStepInputs Result;
		Result.SimulationFrame = State.SimulationFrame;
		Result.ConfigVersion = State.ConfigVersion;
		Result.MaterialVersion = State.MaterialVersion;
		Result.EnvironmentRevision = State.EnvironmentRevision;
		return Result;
	}
	inline bool ExactlyEqual(const FBallState& A, const FBallState& B)
	{
		TArray<uint8> Left, Right;
		return FBallStateCodec::Save(A, Left) && FBallStateCodec::Save(B, Right) && Left == Right;
	}
	inline FBallState RichState()
	{
		FBallState Result = State(200);
		Result.Identity.Generation = 6;
		Result.ConfigVersion = 2;
		Result.MaterialVersion = 3;
		Result.EnvironmentRevision = 4;
		Result.Position = FVector(11.25, -38, 100);
		Result.LinearVelocity = FVector(0.125, -2, 8);
		Result.Rotation = FQuat(FVector::UpVector, 0.75);
		Result.AngularVelocity = FVector(1, 2, 3);
		Result.Activity = EBallActivityState::Resting;
		Result.SleepDwellTime = 0.375;
		Result.bHasSupport = true;
		Result.SupportCollider = {42, 8};
		Result.SupportNormal = FVector::UpVector;
		Result.SupportVelocity = FVector(2, 0, 0);
		Result.DeterministicSeed = 0xfedcba9876543210ull;
		Result.NextEventOrdinal = 92;
		Result.ContactCount = BallLimits::Contacts;
		for (int32 I = 0; I < Result.ContactCount; ++I)
		{
			FBallContactState& Contact = Result.Contacts[I];
			Contact.Collider = {static_cast<uint64>(100 + I), 2};
			Contact.Episode = I + 1;
			Contact.LastImpactFrame = 180 + I;
			Contact.Normal = FVector::RightVector;
			Contact.ContactPoint = FVector(I, I * 2, I * 3);
			Contact.AccumulatedNormalImpulse = I + 0.5;
			Contact.AccumulatedTangentImpulse = FVector(0.1 * I, -0.3, 0.9);
			Contact.Separation = -0.01 * I;
			Contact.bTouching = I % 2 == 0;
			Contact.bSupporting = I % 3 == 0;
			Contact.bRestitutionApplied = true;
		}
		Result.ForceStateCount = BallLimits::ForceProviders;
		for (int32 I = 0; I < Result.ForceStateCount; ++I)
		{
			Result.ForceStates[I].ProviderId = I + 1;
			Result.ForceStates[I].Version = I + 2;
			Result.ForceStates[I].Counter = 50 + I;
			for (int32 J = 0; J < 4; ++J) { Result.ForceStates[I].Values[J] = I * 10 + J + 0.25; }
		}
		return Result;
	}
}
