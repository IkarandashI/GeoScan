#pragma once

#include "CoreMinimal.h"

/**
 * Deterministic PCG32 random number generator.
 *
 * Scenario generation is seeded from the scenario config ("Random Seed" requirement),
 * so the RNG must produce an identical sequence for an identical seed on every
 * platform and build — engine-provided RNGs do not guarantee that, this one does.
 */
class FPCG32
{
public:
	FPCG32() = default;

	FPCG32(uint64 Seed, uint64 Stream = DefaultStream)
	{
		Init(Seed, Stream);
	}

	void Init(uint64 Seed, uint64 Stream);

	uint32 NextUInt32();

	/** Uniform double in [0, 1). */
	double NextUnitDouble();

	/** Uniform double in [Min, Max). */
	double Range(double Min, double Max);

	/** Uniform int in [MinInclusive, MaxInclusive]. */
	int32 RangeInt(int32 MinInclusive, int32 MaxInclusive);

	/** True with the given probability. */
	bool Chance(double Probability);

	static constexpr uint64 DefaultStream = 0xDA3E39CB94B95BDBULL;

private:
	uint64 State = 0;
	uint64 Inc = 0;
};
