#pragma once
#include "CoreMinimal.h"

struct FOURSQUAREVR_API FBallClockEstimate
{
	double ReferenceLocal = 0, RemoteMinusLocal = 0, Drift = 0, RTT = 0, Jitter = 0, Uncertainty = 1;
	uint32 Version = 0;
	bool IsValid() const;
	double RemoteToLocal(double Remote) const;
};
/** Server-initiated four-timestamp exchanges. Remote clock is never assumed to equal the server clock. */
class FOURSQUAREVR_API FBallNetworkClock
{
  public:
	bool Observe(double LocalSend, double RemoteReceive, double RemoteSend, double LocalReceive,
				 double MaximumDriftPPM = 500);
	FBallClockEstimate GetEstimate(double Now) const;
	bool IsReady() const
	{
		return Count >= 2 && Estimate.Version != 0;
	}
	void Reset()
	{
		*this = FBallNetworkClock();
	}

  private:
	struct FExchange
	{
		double Mid = 0, Offset = 0, RTT = 0;
	} Exchanges[16];
	int32 Count = 0, Head = 0;
	FBallClockEstimate Estimate;
	double MaximumDrift = .0005;
	double DriftAnchorLocal = 0, DriftAnchorOffset = 0;
	double LastReceive = -1;
};
struct FOURSQUAREVR_API FBallTokenBucket
{
	double Tokens = 8, Last = -1;
	bool Consume(double Now, double Rate, double Cost = 1, double Capacity = 8);
};
