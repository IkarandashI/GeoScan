#pragma once

#include "CoreMinimal.h"
#include "Gen/SystemTypes.h"
#include "Sim/SpacecraftState.h"

/** Mission-level hard limits; zero means unlimited. */
struct FSpacecraftFlightLimits
{
	double MaxFlightTime = 0.0; // s
	double MaxSpeed = 0.0;      // m/s, heliocentric
};

enum class EMissionFailure : uint8
{
	None,
	Collision,  // spacecraft crossed a body's surface
	Fuel,       // a planned maneuver needs more fuel than available
	TimeLimit,  // Max Flight Time exceeded
	SpeedLimit  // Max Speed exceeded
};

/** One instantaneous delta-v at mission time, with a human-readable note. */
struct FManeuverEvent
{
	double Time = 0.0;            // s from mission start
	FVector DeltaV = FVector::ZeroVector; // m/s, world axes
	FName Note;
};

/** Result of a planner run (M5 fills metrics) or of a manual plan. */
struct FMissionPlan
{
	TArray<FManeuverEvent> Maneuvers; // sorted by Time ascending
	double TotalDeltaV = 0.0;         // m/s
	double FlightTime = 0.0;          // s until boundary
	double RiskScore = 0.0;           // [0..1]
	bool IsValid() const { return !Maneuvers.IsEmpty(); }
};

struct FPlanExecutionResult
{
	int32 ExecutedCount = 0;
	EMissionFailure Failure = EMissionFailure::None;
};

namespace SpacecraftSim
{
	/**
	 * Applies every maneuver whose time has come (in order, at their exact event
	 * times — safe under large time warps). Pending must be sorted by Time.
	 * Stops with Failure=Fuel when a maneuver cannot be paid for.
	 */
	FPlanExecutionResult ExecuteDueManeuvers(FSpacecraftState& State, const FPlanetarySystem& System,
		TArray<FManeuverEvent>& PendingManeuvers, double SimTime);

	/**
	 * Hard mission constraints. Collision is checked at the sampled instant only —
	 * under extreme warp a fly-through between samples can slip past; the planner
	 * validates every flyby analytically, so this is a runtime safety net.
	 */
	EMissionFailure CheckFlightConstraints(const FSpacecraftState& State, const FPlanetarySystem& System,
		double Time, const FSpacecraftFlightLimits& Limits);
}
