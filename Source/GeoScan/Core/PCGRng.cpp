#include "Core/PCGRng.h"

void FPCG32::Init(uint64 Seed, uint64 Stream)
{
	State = 0;
	Inc = (Stream << 1) | 1;
	NextUInt32();
	State += Seed;
	NextUInt32();
}

uint32 FPCG32::NextUInt32()
{
	const uint64 OldState = State;
	State = OldState * 6364136223846793005ULL + Inc;
	const uint32 Xorshifted = static_cast<uint32>(((OldState >> 18) ^ OldState) >> 27);
	const uint32 Rot = static_cast<uint32>(OldState >> 59);
	return (Xorshifted >> Rot) | (Xorshifted << ((32 - Rot) & 31));
}

double FPCG32::NextUnitDouble()
{
	return static_cast<double>(NextUInt32()) / 4294967296.0; // 2^32
}

double FPCG32::Range(double Min, double Max)
{
	return Min + (Max - Min) * NextUnitDouble();
}

int32 FPCG32::RangeInt(int32 MinInclusive, int32 MaxInclusive)
{
	return MinInclusive + static_cast<int32>(NextUInt32() % static_cast<uint32>(MaxInclusive - MinInclusive + 1));
}

bool FPCG32::Chance(double Probability)
{
	return NextUnitDouble() < Probability;
}
