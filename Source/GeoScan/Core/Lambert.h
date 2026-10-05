#pragma once

#include "CoreMinimal.h"

/**
 * Universal-variable Lambert solver (Vallado, "Fundamentals of Astrodynamics", Algorithm 57).
 *
 * Answers: "which pair of velocity vectors puts a spacecraft at Position1 now and at
 * Position2 exactly DeltaTime seconds later?" — the building block of every
 * body-to-body transfer in the route planner.
 *
 * Single-revolution (direct) transfers only. The transfer angle is the short way
 * (< 180 degrees) when bShortWay is true, otherwise the long way. Returns false for
 * degenerate geometry (collinear positions — the classic 180-degree singularity).
 */
namespace Lambert
{
	bool Solve(const FVector& Position1, const FVector& Position2, double Mu, double DeltaTime, bool bShortWay,
		FVector& OutVelocity1, FVector& OutVelocity2);
}
