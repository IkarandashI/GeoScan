#pragma once

#include "CoreMinimal.h"
#include "Gen/SystemTypes.h"

/**
 * Spacecraft state and patched-conics state transitions (SOI switches, impulses).
 * Domain layer: no UObject — UE-facing config maps into FSpacecraftFlightParams.
 */
struct FSpacecraftFlightParams
{
	FName StartingBodyId;
	double StartingOrbitRadius = 1.5e8; // m
	double InitialOrbitPhase = 0.0;     // rad
	double DryMass = 1000.0;            // kg
	double FuelMass = 4000.0;           // kg
	double MaxThrust = 250000.0;        // N
	double Isp = 3500.0;                // s
};

struct FSpacecraftState
{
	FName DominantBodyId;

	// Conic snapshot: position/velocity relative to the dominant body at Epoch.
	double Epoch = 0.0;
	FVector Position = FVector::ZeroVector; // m
	FVector Velocity = FVector::ZeroVector; // m/s

	// Fixed for the whole mission except fuel (and hence total mass) — task requirement.
	double DryMass = 0.0;
	double FuelMass = 0.0;
	double MaxThrust = 0.0;
	double Isp = 0.0;

	double TotalMass() const { return DryMass + FuelMass; }
	double DeltaVRemaining() const;
};

namespace SpacecraftSim
{
	/** Circular parking orbit around the starting body, in that body's orbital plane. */
	FSpacecraftState MakeInitialState(const FPlanetarySystem& System, const FSpacecraftFlightParams& Params);

	/** Star-centered world state at mission time (analytic propagation of the snapshot). */
	bool GetWorldState(const FSpacecraftState& State, const FPlanetarySystem& System, double Time,
		FVector& OutPosition, FVector& OutVelocity);

	/**
	 * Switches the dominant body when the spacecraft crosses a sphere of influence
	 * (upwards on exit, downwards on entering a child's SOI), re-snapshotting the state.
	 */
	void UpdateDominantBody(FSpacecraftState& State, const FPlanetarySystem& System, double Time);

	/**
	 * Re-snapshots at Time and applies an instantaneous delta-v.
	 * Returns false and leaves velocity untouched when there is not enough fuel —
	 * a maneuver can never burn more fuel than available (task constraint).
	 */
	bool ApplyImpulse(FSpacecraftState& State, const FPlanetarySystem& System, double Time, const FVector& DeltaV);
}
