#include "Ball/HistoricalBallCollisionWorld.h"
#include "Ball/BallHandCollisionSolver.h"
const FBallMotionFrame *FHistoricalBallCollisionWorld::GetMotionFrame(const FBallStepInputs &Inputs) const
{
	if (PendingFrame && PendingFrame->Frame == Inputs.SimulationFrame &&
		PendingFrame->Timebase == Inputs.MotionTimebase && PendingFrame->Revision == Inputs.MotionRevision)
	{
		return bPendingValid ? PendingFrame : nullptr;
	}
	return MotionHistory.Find(Inputs.SimulationFrame, Inputs.MotionTimebase, Inputs.MotionRevision);
}
bool FHistoricalBallCollisionWorld::ValidateInputs(const FBallStepInputs &Inputs) const
{
	const auto *F = GetMotionFrame(Inputs);
	return F && F->IsValid() && F->EnvironmentRevision == Inputs.EnvironmentRevision &&
		   F->EnvironmentRevision == GetRevision();
}
EBallQueryResult FHistoricalBallCollisionWorld::SweepSphere(const FBallCollisionQuery &Q, FBallContact &C) const
{
	FBallContactBatch B;
	const auto R = GatherContacts(Q, B);
	if (R == EBallQueryResult::Contact)
	{
		C = B.Contacts[0];
	}
	return R;
}
EBallQueryResult FHistoricalBallCollisionWorld::GatherContacts(const FBallCollisionQuery &Q,
															   FBallContactBatch &Output) const
{
	const auto R = StaticWorld.GatherContacts(Q, Output);
	if (R != EBallQueryResult::Clear && R != EBallQueryResult::Contact)
	{
		return R;
	}
	FBallStepInputs Inputs;
	Inputs.SimulationFrame = Q.StepFrame;
	Inputs.MotionTimebase = Q.MotionTimebase;
	Inputs.MotionRevision = Q.MotionRevision;
	const auto *F = GetMotionFrame(Inputs);
	if (!F || F->EnvironmentRevision != GetRevision())
	{
		Output.Count = 0;
		return EBallQueryResult::InvalidQuery;
	}
	if ((Q.Time.Frame != Q.StepFrame && !(Q.bProximity && Q.Time.Frame == Q.StepFrame + 1 && Q.Time.Fraction == 0)) ||
		(!Q.bProximity &&
		 FBallHandCollisionSolver::NextBoundary(*F, Q.Time.Fraction, Q.EndFraction) < Q.EndFraction - 1.e-10))
	{
		Output.Count = 0;
		return EBallQueryResult::InvalidQuery;
	}
	for (int32 I = 0; I < F->ColliderCount; ++I)
	{
		if (Q.IgnoredMotionMask & (1 << I))
		{
			continue;
		}
		FBallContact C;
		if (FBallHandCollisionSolver::Sweep(*F, F->Colliders[I], Q, C) && !Output.Add(C, Q))
		{
			return EBallQueryResult::CapacityExceeded;
		}
	}
	Output.Sort();
	return Output.Count ? EBallQueryResult::Contact : EBallQueryResult::Clear;
}
