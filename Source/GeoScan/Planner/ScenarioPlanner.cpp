#include "Planner/ScenarioPlanner.h"

#include "Config/ScenarioConfig.h"
#include "Planner/RouteCriteria.h"

namespace ScenarioPlanner
{

TFunction<double(const FRouteMetrics&)> MakeCostEvaluator(const TArray<FCriterionWeight>& Criteria, UObject* Outer)
{
	struct FEntry
	{
		TObjectPtr<URouteCriterion> Criterion;
		double Weight = 0.0;
	};

	TArray<FEntry> Entries;
	double WeightSum = 0.0;
	for (const FCriterionWeight& Spec : Criteria)
	{
		URouteCriterion* Criterion = URouteCriterion::CreateById(Outer, Spec.CriterionId);
		if (Criterion)
		{
			Entries.Add({ Criterion, Spec.Weight });
			WeightSum += FMath::Abs(Spec.Weight);
		}
	}

	return [Entries, WeightSum](const FRouteMetrics& Metrics) -> double
	{
		if (Entries.IsEmpty() || WeightSum <= 0.0)
		{
			// Equal thirds keeps the planner usable with a broken criteria list.
			return (FMath::Min(Metrics.TotalDeltaV / FMath::Max(Metrics.DeltaVBudget, 1.0), 2.0)
				+ FMath::Min(Metrics.FlightTime / FMath::Max(Metrics.MaxFlightTime, 1.0), 2.0)
				+ Metrics.RiskScore) / 3.0;
		}
		double Cost = 0.0;
		for (const FEntry& Entry : Entries)
		{
			Cost += Entry.Weight * FMath::Clamp(Entry.Criterion->Evaluate(Metrics), 0.0, 2.0);
		}
		return Cost;
	};
}

FPlanRequest MakeRequest(const UScenarioConfig* Config, const FPlanetarySystem& System, const FSpacecraftState& InitialState)
{
	FPlanRequest Request;
	Request.System = System;
	Request.InitialState = InitialState;
	Request.Limits.MaxFlightTime = Config->Spacecraft.MaxFlightTime;
	Request.Limits.MaxSpeed = Config->Spacecraft.MaxSpeed;
	Request.RiskParams = Config->Optimization.RiskParams;

	Request.Search.BeamWidth = Config->Optimization.BeamWidth;
	Request.Search.MaxHops = Config->Optimization.MaxHops;
	Request.Search.WaitSteps = Config->Optimization.DepartureWindowSteps;
	Request.Search.FlightTimeSteps = Config->Optimization.FlightTimeSteps;
	Request.Search.MinLegFlightDays = Config->Optimization.MinLegFlightDays;
	Request.Search.MaxLegFlightDays = Config->Optimization.MaxLegFlightDays;
	Request.Search.MaxDepartureWaitDays = Config->Optimization.MaxDepartureWaitDays;
	Request.Search.PeriapsisSafetyFactor = Config->Optimization.PeriapsisSafetyFactor;

	for (const FCriterionWeight& Spec : Config->Optimization.Criteria)
	{
		FCriterionSpec OutSpec;
		OutSpec.Id = Spec.CriterionId;
		OutSpec.Weight = Spec.Weight;
		Request.Criteria.Add(OutSpec);
	}

	Request.CostEvaluator = MakeCostEvaluator(Config->Optimization.Criteria, const_cast<UScenarioConfig*>(Config));
	return Request;
}

FPlanningResult PlanScenario(const UScenarioConfig* Config, const FPlanetarySystem& System, const FSpacecraftState& InitialState)
{
	return RoutePlanner::PlanRoute(MakeRequest(Config, System, InitialState));
}

} // namespace ScenarioPlanner
