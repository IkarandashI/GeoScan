#pragma once

#include "CoreMinimal.h"
#include "Gen/SystemTypes.h"

/**
 * Deterministic procedural generation of a planetary system from a seed.
 * The same parameters (including the seed) always produce the identical system —
 * same PCG32 stream, same math, no platform or launch-order dependence.
 */
struct FSystemGenParams
{
	uint64 Seed = 12345;

	int32 NumPlanets = 6;

	// Star.
	double StarMass = 1.989e30;   // kg, ~1 solar mass
	double StarRadius = 6.96e8;   // m

	// Planet orbits are spaced geometrically between min and max radius.
	double MinOrbitalRadius = 6.0e10;   // m, ~0.4 AU
	double MaxOrbitalRadius = 7.5e12;   // m, ~50 AU

	// Planet properties are drawn log-uniformly from these ranges.
	double PlanetMassMin = 1.0e24;   // kg, ~0.17 Earth
	double PlanetMassMax = 1.9e27;   // kg, ~1 Jupiter
	double PlanetRadiusMin = 2.4e6;  // m
	double PlanetRadiusMax = 5.0e7;  // m

	// Orbit planes: coplanar system by default (decision Q3).
	bool bInclinations = false;
	double MaxInclinationDeg = 8.0;

	// System boundary: the mission succeeds at this distance from the star.
	ESystemBoundaryMode BoundaryMode = ESystemBoundaryMode::FixedDistance;
	double FixedBoundaryDistance = 1.8e13; // m, ~120 AU
	double BoundaryK = 1.5;                // boundary = K * outermost orbit radius
};

namespace SystemGen
{
	/** Generates the system and finalizes all computed fields (Mu, SOI, rates). */
	FPlanetarySystem Generate(const FSystemGenParams& Params);
}
