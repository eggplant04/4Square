#pragma once
#include "Ball/BallCollisionWorld.h"
#include "Ball/BallMotionHistory.h"

/** Engine static world plus immutable recorded trajectories; no UObject access on the dynamic side. */
class FOURSQUAREVR_API FHistoricalBallCollisionWorld final : public IBallCollisionWorld
{
  public:
	FHistoricalBallCollisionWorld(const IBallCollisionWorld &Static, const FBallMotionHistory &History,
								  const FBallMotionFrame *Pending = nullptr)
		: StaticWorld(Static), MotionHistory(History), PendingFrame(Pending),
		  bPendingValid(!Pending || Pending->IsValid())
	{
	}
	virtual uint32 GetRevision() const override
	{
		return StaticWorld.GetRevision();
	}
	virtual bool ValidateInputs(const FBallStepInputs &Inputs) const override;
	virtual const FBallMotionFrame *GetMotionFrame(const FBallStepInputs &Inputs) const override;
	virtual EBallQueryResult SweepSphere(const FBallCollisionQuery &Query, FBallContact &Contact) const override;
	virtual EBallQueryResult GatherContacts(const FBallCollisionQuery &Query, FBallContactBatch &Output) const override;

  private:
	const IBallCollisionWorld &StaticWorld;
	const FBallMotionHistory &MotionHistory;
	const FBallMotionFrame *PendingFrame;
	bool bPendingValid;
};
