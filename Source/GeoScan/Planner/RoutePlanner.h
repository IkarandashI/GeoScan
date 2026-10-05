#pragma once

#include "CoreMinimal.h"
#include "Gen/SystemTypes.h"
#include "Planner/RiskModel.h"
#include "Sim/MissionPlan.h"
#include "Sim/SpacecraftState.h"

/** Search-space knobs; all of them come from the scenario config. */
struct FPlannerSearchParams
{
	int32 BeamWidth = 6;
	int32 MaxHops = 4;
	int32 WaitSteps = 6;         // departure window grid (parking-orbit phase)
	int32 FlightTimeSteps = 6;   // per-leg time-of-flight grid
	double MinLegFlightDays = 20.0;
	double MaxLegFlightDays = 4000.0;
	double MaxDepartureWaitDays = 600.0;
	/** Planned flyby periapsides stay above BodyRadius * factor. */
	double PeriapsisSafetyFactor = 1.5;
};

/** One weighted criterion entry — mirrors the config's {CriterionId, Weight}. */
struct FCriterionSpec
{
	FName Id;
	double Weight = 0.0;
};

struct FPlanRequest
{
	FPlanetarySystem System;
	FSpacecraftState InitialState;   // parking-orbit snapshot
	FSpacecraftFlightLimits Limits;
	FRiskModelParams RiskParams;
	FPlannerSearchParams Search;
	TArray<FCriterionSpec> Criteria;

	/**
	 * Cost = Sum(Weight * normalized criterion) in [0..]. Injected by the caller
	 * (UE-side bridge instantiates URouteCriterion objects) so this planner stays
	 * free of UObject types; empty evaluator falls back to equal thirds.
	 */
	TFunction<double(const struct FRouteMetrics&)> CostEvaluator;
};

struct FPlanningResult
{
	bool bSuccess = false;
	FString FailReason;
	FMissionPlan Plan;
	double TotalDeltaV = 0.0; // m/s
	double FlightTime = 0.0;  // s
	double RiskScore = 0.0;   // [0..1]
	double Cost = 0.0;
	TArray<FName> RouteBodies; // intermediate flybies in order
};

namespace RoutePlanner
{
	/**
	 * Beam search over sequences of bodies: departure burn from the parking orbit,
	 * Lambert legs with powered gravity assists (turn-angle feasibility + magnitude
	 * patch dV), unpowered assist escapes to the system boundary, and direct
	 * powered escapes. The winning route is refined (exact parking point, B-plane
	 * aim offsets on flybys) before being emitted as an executable FMissionPlan.
	 */
	FPlanningResult PlanRoute(const FPlanRequest& Request);
}
