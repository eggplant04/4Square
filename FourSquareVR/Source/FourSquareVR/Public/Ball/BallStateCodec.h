#pragma once

#include "Ball/BallPhysicsTypes.h"

/** Lossless, versioned checkpoint persistence, not the future quantized network snapshot protocol. */
struct FOURSQUAREVR_API FBallStateCodec
{
	static constexpr uint16 FormatVersion = 1;
	static constexpr int32 MaximumEncodedBytes = 8192;
	/** Failed operations leave the caller's destination unchanged. Encoding is outside the simulation hot path. */
	static bool Save(const FBallState& State, TArray<uint8>& OutBytes);
	static bool Load(TConstArrayView<uint8> Bytes, FBallState& OutState);
};
