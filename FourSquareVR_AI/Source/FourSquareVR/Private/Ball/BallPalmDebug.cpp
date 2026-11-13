#include "Ball/BallPalmDebug.h"
#include "DrawDebugHelpers.h"
void FBallPalmDebug::Draw(UWorld *World, const FTransform &T, const FVector &H, double R, FColor Color)
{
#if ENABLE_DRAW_DEBUG
	const FVector Core = H - FVector(R);
	auto Line = [&](const FVector &A, const FVector &B) {
		DrawDebugLine(World, T.TransformPosition(A), T.TransformPosition(B), Color);
	};
	// Circular edge cross-sections at both ends, connected by exact cylinder generators.
	for (int32 Axis = 0; Axis < 3; ++Axis)
	{
		const int32 A = (Axis + 1) % 3, B = (Axis + 2) % 3;
		for (int32 SA : {-1, 1})
			for (int32 SB : {-1, 1})
			{
				FVector Previous[2];
				for (int32 J = 0; J <= 4; ++J)
				{
					const double Angle = J * PI / 8;
					FVector P = FVector::ZeroVector;
					P[A] = SA * (Core[A] + R * FMath::Cos(Angle));
					P[B] = SB * (Core[B] + R * FMath::Sin(Angle));
					FVector Low = P, High = P;
					Low[Axis] = -Core[Axis];
					High[Axis] = Core[Axis];
					Line(Low, High);
					if (J)
					{
						Line(Previous[0], Low);
						Line(Previous[1], High);
					}
					Previous[0] = Low;
					Previous[1] = High;
				}
			}
	}
	// Corner octant latitude curves on the actual rounding spheres.
	for (int32 X : {-1, 1})
		for (int32 Y : {-1, 1})
			for (int32 Z : {-1, 1})
			{
				const FVector Corner = Core * FVector(X, Y, Z);
				for (int32 I = 1; I < 4; ++I)
				{
					FVector Previous;
					const double Phi = I * PI / 8;
					for (int32 J = 0; J <= 4; ++J)
					{
						const double Theta = J * PI / 8;
						const FVector P =
							Corner + R * FVector(X * FMath::Cos(Theta) * FMath::Sin(Phi),
												 Y * FMath::Sin(Theta) * FMath::Sin(Phi), Z * FMath::Cos(Phi));
						if (J)
							Line(Previous, P);
						Previous = P;
					}
				}
			}
	DrawDebugLine(World, T.GetLocation(), T.GetLocation() + T.GetRotation().GetAxisZ() * 10, FColor::Blue);
#endif
}
