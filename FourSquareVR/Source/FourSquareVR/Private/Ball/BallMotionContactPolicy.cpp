#include "Ball/BallMotionContactPolicy.h"
#include "Ball/BallHandCollisionSolver.h"
#include "Ball/BallCollisionSolver.h"

namespace
{
FBallMotionPairState *Pair(FBallState &S, const FBallColliderId &Id)
{
	for (int32 I = 0; I < S.MotionPairCount; ++I)
	{
		if (S.MotionPairs[I].Collider == Id)
		{
			return &S.MotionPairs[I];
		}
	}
	return nullptr;
}
bool Dynamic(const FBallContact &C)
{
	return C.Category == EBallContactCategory::Hand || C.Category == EBallContactCategory::Kinematic;
}
} // namespace
void FBallMotionContactPolicy::RetireAbsent(FBallState &S, const FBallMotionFrame &F)
{
	for (int32 I = S.MotionPairCount - 1; I >= 0; --I)
	{
		bool Present = false;
		for (int32 J = 0; J < F.ColliderCount; ++J)
		{
			Present |= S.MotionPairs[I].Collider == F.Colliders[J].Id;
		}
		if (!Present)
		{
			for (int32 J = I + 1; J < S.MotionPairCount; ++J)
			{
				S.MotionPairs[J - 1] = S.MotionPairs[J];
			}
			--S.MotionPairCount;
		}
	}
}
uint16 FBallMotionContactPolicy::PrepareQuery(FBallState &S, const FBallMotionFrame &F, double T, double Radius,
											  double Release)
{
	uint16 Mask = 0;
	for (int32 I = 0; I < F.ColliderCount; ++I)
	{
		const auto &C = F.Colliders[I];
		if (!FBallHandCollisionSolver::SegmentAt(F, C, T))
		{
			continue;
		}
		auto *P = Pair(S, C.Id);
		const double Separation = FBallHandCollisionSolver::Separation(F, C, T, S.Position, Radius);
		if (!P)
		{
			check(S.MotionPairCount < BallLimits::MotionPairs);
			int32 Insert = 0;
			while (Insert < S.MotionPairCount && (S.MotionPairs[Insert].Collider.Value < C.Id.Value ||
												  (S.MotionPairs[Insert].Collider.Value == C.Id.Value &&
												   S.MotionPairs[Insert].Collider.Generation < C.Id.Generation)))
			{
				++Insert;
			}
			for (int32 J = S.MotionPairCount; J > Insert; --J)
			{
				S.MotionPairs[J] = S.MotionPairs[J - 1];
			}
			++S.MotionPairCount;
			P = &S.MotionPairs[Insert];
			*P = {};
			P->Collider = C.Id;
			P->bArmed = Separation > Release;
			P->bBlocked = !P->bArmed; // An unknown/new lifetime overlapping the ball cannot invent a past strike or
									  // large MTD push.
			P->BlockReason = P->bBlocked ? EBallMotionBlockReason::InitialOverlap : EBallMotionBlockReason::None;
		}
		if (Separation > Release)
		{
			P->bArmed = true;
			P->bBlocked = false;
			P->BlockReason = EBallMotionBlockReason::None;
		}
		if (P->bBlocked)
		{
			Mask |= uint16(1 << I);
		}
	}
	return Mask;
}
void FBallMotionContactPolicy::FilterConflicts(FBallState &S, FBallContactBatch &Batch, double Skin,
											   FBallStepOutput &Output)
{
	FBallContactBatch Accepted;
	// Static geometry is mandatory. Dynamic constraints follow stable collider/feature order.
	for (int32 I = 0; I < Batch.Count; ++I)
	{
		if (!Dynamic(Batch.Contacts[I]))
		{
			Accepted.Contacts[Accepted.Count++] = Batch.Contacts[I];
		}
	}
	for (int32 I = 0; I < Batch.Count; ++I)
	{
		auto C = Batch.Contacts[I];
		if (!Dynamic(C))
		{
			continue;
		}
		auto *P = Pair(S, C.Collider);
		if (!P)
		{
			continue;
		}
		if (P->bBlocked)
		{
			continue;
		}
		C.bAllowImpact = P->bArmed || (P->Episode > 0 && S.NextEventOrdinal > P->EventCounterAfterImpact);
		Accepted.Contacts[Accepted.Count++] = C;
		FBallState Scratch = S;
		if (!FBallCollisionSolver::HasFeasibleVelocity(Accepted) ||
			!FBallCollisionSolver::Separate(Scratch, Accepted, Skin))
		{
			--Accepted.Count;
			P->bBlocked = true;
			P->bArmed = false;
			P->BlockReason = EBallMotionBlockReason::ConflictingContact;
			++Output.Diagnostics.RejectedMotionContacts;
		}
	}
	Accepted.Sort();
	Batch = Accepted;
}
void FBallMotionContactPolicy::CompleteResponse(FBallState &S, const FBallContactBatch &Batch, int32 FirstEvent,
												FBallStepOutput &Output)
{
	for (int32 I = 0; I < Batch.Count; ++I)
	{
		const auto &C = Batch.Contacts[I];
		auto *P = Dynamic(C) ? Pair(S, C.Collider) : nullptr;
		if (!P || !C.bAllowImpact)
		{
			continue;
		}
		bool HasImpulse = false;
		for (int32 J = 0; J < S.ContactCount; ++J)
		{
			HasImpulse |= S.Contacts[J].Collider == C.Collider && S.Contacts[J].AccumulatedNormalImpulse > 1.e-9;
		}
		if (!HasImpulse)
		{
			continue;
		}
		P->bArmed = false;
		P->EventCounterAfterImpact = S.NextEventOrdinal;
		if (P->Episode < MAX_uint32)
		{
			++P->Episode;
		}
		for (int32 J = FirstEvent; J < Output.EventCount; ++J)
		{
			if (Output.Events[J].Collider == C.Collider)
			{
				Output.Events[J].Episode = P->Episode;
			}
		}
	}
}
