#include "Ball/BallPhysicsTypes.h"

bool FBallContactState::IsValid() const
{
	return Collider.IsValid() && Episode != 0 && !Normal.ContainsNaN() && Normal.IsUnit(1.e-6) &&
		   !ContactPoint.ContainsNaN() && !AccumulatedTangentImpulse.ContainsNaN() &&
		   FMath::IsFinite(AccumulatedNormalImpulse) && AccumulatedNormalImpulse >= 0 && FMath::IsFinite(Separation);
}

bool FBallForceState::IsValid() const
{
	if (ProviderId == 0 || Version == 0)
	{
		return false;
	}
	for (const double Value : Values)
	{
		if (!FMath::IsFinite(Value))
		{
			return false;
		}
	}
	return true;
}

bool FBallState::IsValid() const
{
	if (!Identity.IsValid() || SimulationFrame == MAX_uint64 || ConfigVersion == 0 || MaterialVersion == 0 ||
		EnvironmentRevision == 0 || Position.ContainsNaN() || LinearVelocity.ContainsNaN() || Rotation.ContainsNaN() ||
		!Rotation.IsNormalized() || AngularVelocity.ContainsNaN() ||
		static_cast<uint8>(Activity) > static_cast<uint8>(EBallActivityState::Resting) ||
		!FMath::IsFinite(SleepDwellTime) || SleepDwellTime < 0 || SupportNormal.ContainsNaN() ||
		!SupportNormal.IsUnit(1.e-6) || SupportVelocity.ContainsNaN() || (bHasSupport && !SupportCollider.IsValid()) ||
		(Activity == EBallActivityState::Resting && !bHasSupport) || ContactCount < 0 ||
		ContactCount > BallLimits::Contacts || ForceStateCount < 0 || ForceStateCount > BallLimits::ForceProviders ||
		MotionPairCount < 0 || MotionPairCount > BallLimits::MotionPairs)
	{
		return false;
	}
	for (int32 I = 0; I < MotionPairCount; ++I)
	{
		if (!MotionPairs[I].Collider.IsValid() || MotionPairs[I].EventCounterAfterImpact > NextEventOrdinal ||
			uint8(MotionPairs[I].BlockReason) > uint8(EBallMotionBlockReason::ConflictingContact) ||
			(MotionPairs[I].bBlocked != (MotionPairs[I].BlockReason != EBallMotionBlockReason::None)))
		{
			return false;
		}
		for (int32 J = 0; J < I; ++J)
		{
			if (MotionPairs[I].Collider == MotionPairs[J].Collider)
			{
				return false;
			}
		}
	}
	for (int32 I = 0; I < ContactCount; ++I)
	{
		if (!Contacts[I].IsValid() || Contacts[I].LastImpactFrame > SimulationFrame)
		{
			return false;
		}
		for (int32 J = 0; J < I; ++J)
		{
			if (Contacts[J].Collider == Contacts[I].Collider && Contacts[J].FeatureId == Contacts[I].FeatureId)
			{
				return false;
			}
		}
	}
	for (int32 I = 0; I < ForceStateCount; ++I)
	{
		if (!ForceStates[I].IsValid())
		{
			return false;
		}
		for (int32 J = 0; J < I; ++J)
		{
			if (ForceStates[J].ProviderId == ForceStates[I].ProviderId)
			{
				return false;
			}
		}
	}
	return true;
}

bool FBallStepInputs::IsValid() const
{
	if (SimulationFrame == MAX_uint64 || ConfigVersion == 0 || MaterialVersion == 0 || EnvironmentRevision == 0 ||
		ForceCount < 0 || ForceCount > BallLimits::ForceProviders || ((MotionTimebase == 0) != (MotionRevision == 0)))
	{
		return false;
	}
	for (int32 I = 0; I < ForceCount; ++I)
	{
		const FBallForceInput &Force = Forces[I];
		if (Force.ProviderId == 0 || Force.LinearAcceleration.ContainsNaN() || Force.AngularAcceleration.ContainsNaN())
		{
			return false;
		}
		if (I > 0 && (Forces[I - 1].Priority > Force.Priority ||
					  (Forces[I - 1].Priority == Force.Priority && Forces[I - 1].ProviderId >= Force.ProviderId)))
		{
			return false;
		}
		for (int32 J = 0; J < I; ++J)
		{
			if (Forces[J].ProviderId == Force.ProviderId)
			{
				return false;
			}
		}
	}
	return true;
}
