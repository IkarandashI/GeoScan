#pragma once

#include "CoreMinimal.h"
#include "Planner/RoutePlanner.h"

struct FCriterionWeight;
class UScenarioConfig;

/**
 * UE-side bridge: scenario config -> planner request. Instantiates URouteCriterion
 * objects from the config's {CriterionId, Weight} list and hides them behind the
 * planner's injected cost evaluator — the planner itself never sees UObjects.
 */
namespace ScenarioPlanner
{
	/** Builds the cost evaluator from config criteria; unknown ids fall back to equal thirds. */
	TFunction<double(const FRouteMetrics&)> MakeCostEvaluator(const TArray<FCriterionWeight>& Criteria, UObject* Outer);

	FPlanRequest MakeRequest(const UScenarioConfig* Config, const FPlanetarySystem& System, const FSpacecraftState& InitialState);

	FPlanningResult PlanScenario(const UScenarioConfig* Config, const FPlanetarySystem& System, const FSpacecraftState& InitialState);
}
