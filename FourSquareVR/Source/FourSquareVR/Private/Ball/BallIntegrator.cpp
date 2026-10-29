#include "Ball/BallIntegrator.h"

bool FBallIntegrator::Kick(FBallState& State, const FVector& LinearAcceleration, const FVector& AngularAcceleration,
	const FBallPhysicsConfig& Config, double DeltaTime)
{
	State.LinearVelocity = (State.LinearVelocity + LinearAcceleration * DeltaTime) * FMath::Exp(-Config.LinearDrag * DeltaTime);
	State.AngularVelocity = (State.AngularVelocity + AngularAcceleration * DeltaTime) * FMath::Exp(-Config.AngularDrag * DeltaTime);
	if (State.LinearVelocity.ContainsNaN() || State.AngularVelocity.ContainsNaN()
		|| !FMath::IsFinite(State.LinearVelocity.SizeSquared()) || !FMath::IsFinite(State.AngularVelocity.SizeSquared())) { return false; }
	if (Config.MaximumLinearSpeed > 0) { State.LinearVelocity = State.LinearVelocity.GetClampedToMaxSize(Config.MaximumLinearSpeed); }
	if (Config.MaximumAngularSpeed > 0) { State.AngularVelocity = State.AngularVelocity.GetClampedToMaxSize(Config.MaximumAngularSpeed); }
	return true;
}

bool FBallIntegrator::Drift(FBallState& State, double DeltaTime)
{
	State.Position += State.LinearVelocity * DeltaTime;
	const double Speed = State.AngularVelocity.Size();
	if (!FMath::IsFinite(Speed * DeltaTime)) { return false; }
	if (Speed > UE_DOUBLE_SMALL_NUMBER)
	{
		State.Rotation = FQuat(State.AngularVelocity / Speed, Speed * DeltaTime) * State.Rotation;
		State.Rotation.Normalize();
	}
	return !State.Position.ContainsNaN() && !State.Rotation.ContainsNaN() && State.Rotation.IsNormalized();
}
