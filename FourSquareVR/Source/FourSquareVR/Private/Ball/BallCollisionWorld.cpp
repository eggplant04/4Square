#include "Ball/BallCollisionWorld.h"

bool FBallContactBatch::Add(const FBallContact& Contact, const FBallCollisionQuery& Query)
{
	if (Contact.Normal.ContainsNaN() || !Contact.Normal.IsUnit(1.e-4) || !Contact.Collider.IsValid() ||
		!Contact.Material.IsValid() || !FMath::IsFinite(Contact.TimeOfImpact) || Contact.TimeOfImpact < 0 ||
		Contact.TimeOfImpact > 1 || !FMath::IsFinite(Contact.PenetrationDepth) || Contact.PenetrationDepth < 0 ||
		!FMath::IsFinite(Contact.Separation) || Contact.Position.ContainsNaN() || Contact.SurfaceVelocity.ContainsNaN())
	{
		return false;
	}
	if (!Query.bProximity && Count > 0)
	{
		double Earliest = Contacts[0].TimeOfImpact;
		for (int32 I = 1; I < Count; ++I)
		{
			Earliest = FMath::Min(Earliest, Contacts[I].TimeOfImpact);
		}
		if (Contact.TimeOfImpact < Earliest)
		{
			int32 Keep = 0;
			for (int32 I = 0; I < Count; ++I)
			{
				if (Contacts[I].TimeOfImpact <= Contact.TimeOfImpact + Query.TOIFractionTolerance)
				{
					Contacts[Keep++] = Contacts[I];
				}
			}
			Count = Keep;
		}
		else if (Contact.TimeOfImpact > Earliest + Query.TOIFractionTolerance)
		{
			return true;
		}
	}
	for (int32 I = 0; I < Count; ++I)
	{
		if (Contacts[I].Collider == Contact.Collider && Contacts[I].FeatureId == Contact.FeatureId)
		{
			if (Contact.PenetrationDepth > Contacts[I].PenetrationDepth)
			{
				Contacts[I] = Contact;
			}
			return true;
		}
	}
	if (Count == BallLimits::Contacts)
	{
		return false;
	}
	Contacts[Count++] = Contact;
	return true;
}

void FBallContactBatch::Sort()
{
	for (int32 I = 1; I < Count; ++I)
	{
		FBallContact Contact = Contacts[I];
		int32 J = I;
		const auto Less = [](const FBallContact& A, const FBallContact& B)
		{
			if (A.Collider.Value != B.Collider.Value)
			{
				return A.Collider.Value < B.Collider.Value;
			}
			if (A.Collider.Generation != B.Collider.Generation)
			{
				return A.Collider.Generation < B.Collider.Generation;
			}
			return A.FeatureId < B.FeatureId;
		};
		while (J > 0 && Less(Contact, Contacts[J - 1]))
		{
			Contacts[J] = Contacts[J - 1];
			--J;
		}
		Contacts[J] = Contact;
	}
}
