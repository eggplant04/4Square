#pragma once
#include "BallTestFixtures.h"
#include "Ball/BallPalmGeometry.h"
#include "Ball/BallHandCollisionSolver.h"
#include "Ball/BallSimulator.h"
#include "Ball/HistoricalBallCollisionWorld.h"
#include "Ball/RecordedBallCollisionWorld.h"
namespace PalmTests
{
inline FBallPhysicsConfig Config()
{
	FBallPhysicsConfig C;
	C.Radius = 1;
	C.Gravity = FVector::ZeroVector;
	C.LinearDrag = C.AngularDrag = C.SurfaceFriction = C.SpinFriction = C.RollingResistance = 0;
	return C;
}
inline FBallMotionFrame Frame(const FVector &Start = FVector::ZeroVector, const FVector &End = FVector::ZeroVector,
							  const FQuat &A = FQuat::Identity, const FQuat &B = FQuat::Identity)
{
	FBallMotionFrame F;
	F.Epoch = BallTests::State().Identity.Epoch;
	FBallColliderMotion C;
	C.Id = BallMotionLimits::HandId(1, EBallHandSide::Left, 1);
	C.Shape = EBallMotionShape::RoundedBox;
	C.Participant = 1;
	C.OriginRevision = 1;
	C.HalfExtents = FVector(4, 6, 1);
	C.RoundRadius = .5;
	C.Material.bOverride = true;
	C.Material.Friction = 0;
	FBallMotionSegment S;
	S.Start = Start;
	S.End = End;
	S.StartOrientation = A;
	S.EndOrientation = B;
	S.FirstSample = 10;
	S.SecondSample = 11;
	check(F.Add(C, MakeArrayView(&S, 1)));
	return F;
}
inline FBallCollisionQuery Query(const FVector &A, const FVector &B, double Radius = 1)
{
	FBallCollisionQuery Q;
	Q.Start = A;
	Q.End = B;
	Q.Radius = Radius;
	return Q;
}
inline EBallStepResult Step(FBallMotionFrame F, FBallState &S, const FBallPhysicsConfig &C, FBallStepOutput &O,
							const IBallCollisionWorld *Static = nullptr)
{
	F.Frame = S.SimulationFrame;
	FRecordedBallCollisionWorld Empty;
	FBallMotionHistory H;
	H.Initialize(2);
	H.Push(F);
	FHistoricalBallCollisionWorld W(Static ? *Static : Empty, H);
	auto I = BallTests::Inputs(S);
	I.MotionTimebase = I.MotionRevision = 1;
	FBallSimulator Sim;
	return Sim.SimulateStep(S, I, C, W, O);
}
inline FBallHandObservation Pose(double T, const FVector &P, const FQuat &Q, uint32 Seq)
{
	FBallHandObservation O;
	O.CaptureSeconds = O.TimelineSeconds = T;
	O.ControllerPosition = P;
	O.ControllerOrientation = Q;
	O.HeadPosition = P + FVector(0, 0, 50);
	O.Sequence = Seq;
	return O;
}
inline FBallMotionFrame Wrist()
{
	auto F =
		Frame(FVector::ZeroVector, FVector::ZeroVector, FQuat(FVector::UpVector, -.3), FQuat(FVector::UpVector, .3));
	F.Colliders[0].HalfExtents = FVector(6, .5, .5);
	F.Colliders[0].RoundRadius = .2;
	return F;
}
} // namespace PalmTests
