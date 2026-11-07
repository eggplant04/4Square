#include "Ball/BallNetworkClock.h"
bool FBallClockEstimate::IsValid() const
{
	return Version && FMath::IsFinite(ReferenceLocal) && FMath::IsFinite(RemoteMinusLocal) && FMath::IsFinite(Drift) &&
		   FMath::Abs(Drift) <= .01 && FMath::IsFinite(RTT) && RTT >= 0 && FMath::IsFinite(Jitter) && Jitter >= 0 &&
		   FMath::IsFinite(Uncertainty) && Uncertainty >= 0;
}
double FBallClockEstimate::RemoteToLocal(double Remote) const
{
	return (Remote - RemoteMinusLocal + Drift * ReferenceLocal) / (1 + Drift);
}
bool FBallNetworkClock::Observe(double A, double B, double C, double D, double MaximumDriftPPM)
{
	if (!FMath::IsFinite(A) || !FMath::IsFinite(B) || !FMath::IsFinite(C) || !FMath::IsFinite(D) || D < A || C < B ||
		D - A > 2 || C - B > .05 || !FMath::IsFinite(MaximumDriftPPM) || MaximumDriftPPM < 0 || MaximumDriftPPM > 10000)
		return false;
	const double RTT = (D - A) - (C - B), Offset = ((B - A) + (C - D)) * .5;
	if (RTT < -1.e-6 || (LastReceive >= 0 && D <= LastReceive))
		return false;
	MaximumDrift = MaximumDriftPPM * 1.e-6;
	const FExchange E = {(A + D) * .5, Offset, FMath::Max(0.0, RTT)};
	if (Estimate.Version &&
		FMath::Abs(E.Offset - (Estimate.RemoteMinusLocal + Estimate.Drift * (E.Mid - Estimate.ReferenceLocal))) >
			FMath::Max(.005,
					   Estimate.Uncertainty + E.RTT * .5 + MaximumDrift * FMath::Abs(E.Mid - Estimate.ReferenceLocal)))
		return false;
	Exchanges[Head] = E;
	Head = (Head + 1) % 16;
	Count = FMath::Min(Count + 1, 16);
	FExchange Best = E;
	for (int32 I = 0; I < Count; ++I)
		if (Exchanges[I].RTT < Best.RTT && D - Exchanges[I].Mid < 8)
			Best = Exchanges[I];
	const double PriorOffset = Estimate.RemoteMinusLocal + Estimate.Drift * (Best.Mid - Estimate.ReferenceLocal);
	if (!Estimate.Version)
	{
		DriftAnchorLocal = Best.Mid;
		DriftAnchorOffset = Best.Offset;
	}
	if (Estimate.Version && Best.Mid > DriftAnchorLocal + 1)
	{
		const double Measured = (Best.Offset - DriftAnchorOffset) / (Best.Mid - DriftAnchorLocal);
		Estimate.Drift = FMath::Lerp(Estimate.Drift, FMath::Clamp(Measured, -MaximumDrift, MaximumDrift), .25);
		DriftAnchorLocal = Best.Mid;
		DriftAnchorOffset = Best.Offset;
	}
	const double PreviousJitter = Estimate.Jitter;
	Estimate.Jitter =
		Estimate.Version
			? FMath::Sqrt(.875 * PreviousJitter * PreviousJitter + .125 * FMath::Square(E.Offset - PriorOffset))
			: 0;
	Estimate.ReferenceLocal = Best.Mid;
	Estimate.RemoteMinusLocal = Best.Offset;
	Estimate.RTT = E.RTT;
	Estimate.Uncertainty = Best.RTT * .5 + .0001 + FMath::Abs(D - Best.Mid) * MaximumDrift;
	++Estimate.Version;
	LastReceive = D;
	return Estimate.IsValid();
}
FBallClockEstimate FBallNetworkClock::GetEstimate(double Now) const
{
	auto E = Estimate;
	if (E.Version)
		E.Uncertainty += FMath::Max(0.0, Now - E.ReferenceLocal) * MaximumDrift;
	return E;
}
bool FBallTokenBucket::Consume(double Now, double Rate, double Cost, double Capacity)
{
	if (!FMath::IsFinite(Now) || !FMath::IsFinite(Rate) || Rate <= 0 || Cost <= 0 || Capacity <= 0 ||
		(Last >= 0 && Now < Last))
		return false;
	if (Last >= 0)
		Tokens = FMath::Min(Capacity, Tokens + (Now - Last) * Rate);
	Last = Now;
	if (Tokens + 1.e-9 < Cost)
		return false;
	Tokens -= Cost;
	return true;
}
