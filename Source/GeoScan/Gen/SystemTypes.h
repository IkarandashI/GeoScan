#pragma once

#include "CoreMinimal.h"

/**
 * Domain model of the planetary system (see Docs/design-discussion.md, sections 2.2-2.3).
 * Plain C++ types, no UObject — the UE-facing config lives in Config/.
 *
 * Hierarchy is parent-linked: the star has no parent, planets orbit the star,
 * and the model admits deeper chains (moons) without changes.
 */
enum class ESystemBoundaryMode : uint8
{
	FixedDistance,
	RelativeToOutermost
};

struct FCelestialBodyDesc
{
	FName Id;
	FName TypeId;      // "Star", "Planet", ... — mapped to a visual class by config
	FName ParentId;    // NAME_None for the star

	double Mass = 0.0;         // kg
	double Mu = 0.0;           // gravitational parameter G*Mass, m^3/s^2
	double Radius = 0.0;       // m

	// Circular Kepler orbit around the parent.
	double OrbitRadius = 0.0;  // m
	double InitialPhase = 0.0; // rad
	double Inclination = 0.0;  // rad, 0 in the coplanar default
	double LAN = 0.0;          // longitude of ascending node, rad

	// Computed at generation (finalize).
	double SoiRadius = 0.0;            // Laplace sphere of influence, m
	double OrbitalAngularRate = 0.0;   // rad/s
	double OrbitalPeriod = 0.0;        // s
	double OrbitalSpeed = 0.0;         // m/s

	/** State relative to the parent at the given mission time. */
	void GetOrbitState(double Time, FVector& OutPosition, FVector& OutVelocity) const;

	/** Basis of an orbital plane: P towards theta=0 along the node line, Q 90 degrees ahead in-plane. */
	static void GetOrbitBasis(double InclinationRad, double LanRad, FVector& OutP, FVector& OutQ);
};

struct FPlanetarySystem
{
	TArray<FCelestialBodyDesc> Bodies; // Bodies[0] is the star by convention
	double SystemBoundary = 0.0;       // m — reaching this distance from the star = mission success

	const FCelestialBodyDesc* FindBody(FName BodyId) const;
	const FCelestialBodyDesc& GetStar() const;

	/** Star-centered world state of a body at mission time (composes the parent chain). */
	bool GetBodyWorldState(FName BodyId, double Time, FVector& OutPosition, FVector& OutVelocity) const;

	/** Largest orbit radius among non-star bodies. */
	double OutermostChildOrbitRadius() const;
};
