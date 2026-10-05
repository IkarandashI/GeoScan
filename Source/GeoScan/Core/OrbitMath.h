#pragma once

#include "CoreMinimal.h"

/**
 * Core astrodynamics for the patched-conics model (see Docs/design-discussion.md, section 2).
 *
 * Units: SI — meters, seconds, kg. Each body carries its gravitational parameter
 * Mu = G * M [m^3/s^2]. Base orbital plane is XY (Z up), motion counter-clockwise
 * seen from +Z.
 */
namespace OrbitMath
{
	/** Standard gravitational acceleration used to convert Isp [s] into effective exhaust velocity. */
	inline constexpr double G0 = 9.80665;

	/** Gravitational constant, m^3/(kg*s^2). */
	inline constexpr double GravitationalConstant = 6.67430e-11;

	/** Stumpff functions C(z), S(z) — shared by Kepler propagation and the Lambert solver. */
	double StumpffC(double Z);
	double StumpffS(double Z);

	/** Speed of a circular orbit of the given radius. */
	double CircularOrbitSpeed(double Mu, double Radius);

	/** Orbital period of an orbit with the given semi-major axis (elliptic orbits). */
	double OrbitalPeriod(double Mu, double SemiMajorAxis);

	/** State (position, velocity) of a body on a circular orbit in the base plane at time t. */
	void CircularOrbitState(double Mu, double Radius, double InitialPhase, double Time,
		FVector& OutPosition, FVector& OutVelocity);

	/** Laplace sphere-of-influence radius of a body orbiting its parent at OrbitRadius. */
	double SphereOfInfluenceRadius(double MuBody, double MuParent, double OrbitRadius);

	/** Specific orbital energy; negative for bound (elliptic) orbits. */
	double SpecificEnergy(const FVector& Position, const FVector& Velocity, double Mu);

	/** Semi-major axis from a state vector; negative for hyperbolic orbits. */
	double SemiMajorAxis(const FVector& Position, const FVector& Velocity, double Mu);

	/**
	 * Universal-variable Kepler propagation: state DeltaTime seconds later, exact for any
	 * conic (circle / ellipse / parabola / hyperbola). Vallado, "Fundamentals of Astrodynamics",
	 * Algorithm 8.
	 */
	void Propagate(const FVector& Position0, const FVector& Velocity0, double Mu, double DeltaTime,
		FVector& OutPosition, FVector& OutVelocity);

	/** Fuel mass [kg] burned to spend DeltaV [m/s] with total mass MassTotal0 and engine Isp [s] (Tsiolkovsky). */
	double FuelBurned(double MassTotal0, double DeltaV, double Isp);

	/** Total delta-v available between the current total mass and the dry mass. */
	double DeltaVBudget(double MassTotal0, double MassDry, double Isp);
}
