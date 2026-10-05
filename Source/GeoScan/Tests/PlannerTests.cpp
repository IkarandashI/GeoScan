#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "Config/ScenarioConfig.h"
#include "Core/OrbitMath.h"
#include "Gen/SystemGenerator.h"
#include "Planner/RiskModel.h"
#include "Planner/RouteCriteria.h"
#include "Planner/ScenarioPlanner.h"
#include "Sim/MissionRunner.h"
#include "Sim/SpacecraftState.h"

namespace
{
	FPlanetarySystem MakePlannerSystem()
	{
		FSystemGenParams Params;
		Params.Seed = 4242;
		Params.NumPlanets = 6;
		return SystemGen::Generate(Params);
	}

	UScenarioConfig* MakePreset(EOptimizationMode Mode)
	{
		UScenarioConfig* Config = NewObject<UScenarioConfig>(GetTransientPackage());
		Config->ApplyPreset(Mode);
		return Config;
	}
}

// ---------------------------------------------------------------- Criteria registry

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlannerCriteriaTest, "GeoScan.Planner.Criteria",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FPlannerCriteriaTest::RunTest(const FString&)
{
	URouteCriterion* Fuel = URouteCriterion::CreateById(GetTransientPackage(), FName(TEXT("Fuel")));
	URouteCriterion* Time = URouteCriterion::CreateById(GetTransientPackage(), FName(TEXT("Time")));
	URouteCriterion* Risk = URouteCriterion::CreateById(GetTransientPackage(), FName(TEXT("Risk")));
	TestTrue("Fuel criterion created", Fuel != nullptr);
	TestTrue("Time criterion created", Time != nullptr);
	TestTrue("Risk criterion created", Risk != nullptr);
	TestTrue("Unknown criterion rejected",
		URouteCriterion::CreateById(GetTransientPackage(), FName(TEXT("Happiness"))) == nullptr);

	FRouteMetrics Metrics;
	Metrics.TotalDeltaV = 5000.0;
	Metrics.DeltaVBudget = 10000.0;
	Metrics.FlightTime = 3.0e8;
	Metrics.MaxFlightTime = 6.0e8;
	Metrics.RiskScore = 0.25;

	TestTrue("Fuel score", FMath::IsNearlyEqual(Fuel->Evaluate(Metrics), 0.5, 1e-9));
	TestTrue("Time score", FMath::IsNearlyEqual(Time->Evaluate(Metrics), 0.5, 1e-9));
	TestTrue("Risk score", FMath::IsNearlyEqual(Risk->Evaluate(Metrics), 0.25, 1e-9));
	return true;
}

// ---------------------------------------------------------------- Risk model

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlannerRiskTest, "GeoScan.Planner.RiskModel",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FPlannerRiskTest::RunTest(const FString&)
{
	FRiskModelParams Params; // default weights 0.5 / 0.3 / 0.2
	const double Radius = 1.0e7;

	// Comfortable, slow pass with a full tank: zero risk.
	TestTrue("Safe pass is risk-free",
		RiskModel::EvaluateFlybyRisk(Params, Radius * Params.SafeAltitudeFactor, Radius, 0.0, 0.0) == 0.0);

	// Surface graze at zero speed with full tank: exactly the proximity weight.
	const double SurfaceRisk = RiskModel::EvaluateFlybyRisk(Params, Radius, Radius, 0.0, 0.0);
	TestTrue("Surface risk equals proximity weight", FMath::IsNearlyEqual(SurfaceRisk, Params.WeightProximity, 1e-9));

	// Risk grows as the pass tightens.
	const double R1 = RiskModel::EvaluateFlybyRisk(Params, Radius * 1.4, Radius, 1.0e4, 0.0);
	const double R2 = RiskModel::EvaluateFlybyRisk(Params, Radius * 1.2, Radius, 1.0e4, 0.0);
	TestTrue("Closer pass is riskier", R2 > R1);

	// Faster is riskier; an empty tank adds the reserve factor.
	TestTrue("Speed adds risk",
		RiskModel::EvaluateFlybyRisk(Params, Radius * 1.3, Radius, 5.0e4, 0.0)
		> RiskModel::EvaluateFlybyRisk(Params, Radius * 1.3, Radius, 1.0e4, 0.0));
	TestTrue("Spent fuel adds risk",
		RiskModel::EvaluateFlybyRisk(Params, Radius * 1.3, Radius, 1.0e4, 0.9)
		> RiskModel::EvaluateFlybyRisk(Params, Radius * 1.3, Radius, 1.0e4, 0.1));

	// Clamped to [0..1].
	TestTrue("Risk saturates at 1",
		RiskModel::EvaluateFlybyRisk(Params, Radius, Radius, 1.0e6, 1.0) == 1.0);
	return true;
}

// ---------------------------------------------------------------- Planning + playback

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlannerModesTest, "GeoScan.Planner.Modes",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FPlannerModesTest::RunTest(const FString&)
{
	const FPlanetarySystem System = MakePlannerSystem();

	struct FModeOutcome
	{
		double TotalDeltaV = 0.0;
		double FlightTime = 0.0;
	};
	TMap<EOptimizationMode, FModeOutcome> Outcomes;

	for (const EOptimizationMode Mode : { EOptimizationMode::FuelEfficient, EOptimizationMode::TimeEfficient, EOptimizationMode::Balanced })
	{
		UScenarioConfig* Config = MakePreset(Mode);
		const FSpacecraftState Initial = SpacecraftSim::MakeInitialState(System, Config->ToFlightParams());
		const double Budget = Initial.DeltaVRemaining();

		const FPlanningResult Planned = ScenarioPlanner::PlanScenario(Config, System, Initial);
		if (!TestTrue(FString::Printf(TEXT("Mode %d: route found"), static_cast<int32>(Mode)), Planned.bSuccess))
		{
			UE_LOG(LogTemp, Error, TEXT("Planner failure: %s"), *Planned.FailReason);
			continue;
		}

		TestTrue(FString::Printf(TEXT("Mode %d: dV within budget"), static_cast<int32>(Mode)),
			Planned.TotalDeltaV <= Budget + 1.0);
		TestTrue(FString::Printf(TEXT("Mode %d: flight time within limit"), static_cast<int32>(Mode)),
			Planned.FlightTime <= Config->Spacecraft.MaxFlightTime);
		TestTrue(FString::Printf(TEXT("Mode %d: plan has maneuvers"), static_cast<int32>(Mode)),
			Planned.Plan.Maneuvers.Num() >= 1);
		TestTrue(FString::Printf(TEXT("Mode %d: risk in [0..1]"), static_cast<int32>(Mode)),
			Planned.RiskScore >= 0.0 && Planned.RiskScore <= 1.0);

		// Playback: the plan must actually fly the mission to the boundary.
		const FMissionRunResult Run = MissionRunner::Run(System, Initial, Planned.Plan,
			[Config]{ FSpacecraftFlightLimits Limits; Limits.MaxFlightTime = Config->Spacecraft.MaxFlightTime; Limits.MaxSpeed = Config->Spacecraft.MaxSpeed; return Limits; }());
		TestTrue(FString::Printf(TEXT("Mode %d: playback reaches boundary (failure %d, %.2f AU)"),
			static_cast<int32>(Mode), static_cast<int32>(Run.Failure), Run.FinalStarDistance / 1.496e11),
			Run.bSuccess);

		FModeOutcome& Outcome = Outcomes.FindOrAdd(Mode);
		Outcome.TotalDeltaV = Planned.TotalDeltaV;
		Outcome.FlightTime = Planned.FlightTime;
	}

	// The two extreme modes must respect their priorities.
	if (const FModeOutcome* Fuel = Outcomes.Find(EOptimizationMode::FuelEfficient))
	{
		if (const FModeOutcome* Time = Outcomes.Find(EOptimizationMode::TimeEfficient))
		{
			TestTrue("Fuel mode spends no more dV than Time mode", Fuel->TotalDeltaV <= Time->TotalDeltaV + 1.0);
			TestTrue("Time mode flies no longer than Fuel mode", Time->FlightTime <= Fuel->FlightTime + 1.0);
		}
	}
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlannerDeterminismTest, "GeoScan.Planner.Determinism",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FPlannerDeterminismTest::RunTest(const FString&)
{
	const FPlanetarySystem System = MakePlannerSystem();
	UScenarioConfig* Config = MakePreset(EOptimizationMode::Balanced);
	const FSpacecraftState Initial = SpacecraftSim::MakeInitialState(System, Config->ToFlightParams());

	const FPlanningResult First = ScenarioPlanner::PlanScenario(Config, System, Initial);
	const FPlanningResult Second = ScenarioPlanner::PlanScenario(Config, System, Initial);

	TestTrue("Both runs succeed", First.bSuccess && Second.bSuccess);
	TestEqual("Same maneuver count", First.Plan.Maneuvers.Num(), Second.Plan.Maneuvers.Num());
	bool bIdentical = First.TotalDeltaV == Second.TotalDeltaV && First.FlightTime == Second.FlightTime;
	for (int32 i = 0; i < First.Plan.Maneuvers.Num() && bIdentical; ++i)
	{
		bIdentical &= First.Plan.Maneuvers[i].Time == Second.Plan.Maneuvers[i].Time
			&& First.Plan.Maneuvers[i].DeltaV.Equals(Second.Plan.Maneuvers[i].DeltaV, 1e-6);
	}
	TestTrue("Identical plans for identical inputs", bIdentical);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPlannerReplanTest, "GeoScan.Planner.ReplanFromFlight",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FPlannerReplanTest::RunTest(const FString&)
{
	const FPlanetarySystem System = MakePlannerSystem();
	UScenarioConfig* Config = MakePreset(EOptimizationMode::Balanced);

	// Fly the original plan until well after the departure burn, then replan
	// from the heliocentric cruise state.
	FSpacecraftState State = SpacecraftSim::MakeInitialState(System, Config->ToFlightParams());
	const FPlanningResult InitialPlan = ScenarioPlanner::PlanScenario(Config, System, State);
	if (!TestTrue("Initial plan exists", InitialPlan.bSuccess))
	{
		return false;
	}

	// Find the departure maneuver time and advance past it.
	double DepartureTime = InitialPlan.Plan.Maneuvers[0].Time;
	const double ReplanTime = DepartureTime + 30.0 * 86400.0; // a month into the cruise

	TArray<FManeuverEvent> Pending = InitialPlan.Plan.Maneuvers;
	FPlanExecutionResult Execution = SpacecraftSim::ExecuteDueManeuvers(State, System, Pending, ReplanTime);
	TestTrue("Departure executed before replan", Execution.ExecutedCount >= 1);
	SpacecraftSim::UpdateDominantBody(State, System, ReplanTime);
	TestEqual("Spacecraft is heliocentric after departure",
		State.DominantBodyId, System.GetStar().Id);

	// Re-snapshot at the replan instant.
	if (const FCelestialBodyDesc* Dominant = System.FindBody(State.DominantBodyId))
	{
		OrbitMath::Propagate(State.Position, State.Velocity, Dominant->Mu, ReplanTime - State.Epoch, State.Position, State.Velocity);
		State.Epoch = ReplanTime;
	}

	const FPlanningResult Replanned = ScenarioPlanner::PlanScenario(Config, System, State);
	TestTrue(FString::Printf(TEXT("Replan from cruise succeeds (%s)"), *Replanned.FailReason), Replanned.bSuccess);
	if (!Replanned.bSuccess)
	{
		return false;
	}
	TestTrue("Replanned dV fits the remaining budget",
		Replanned.TotalDeltaV <= State.DeltaVRemaining() + 1.0);

	// The remaining mission must still reach the boundary.
	FSpacecraftFlightLimits Limits;
	Limits.MaxFlightTime = Config->Spacecraft.MaxFlightTime;
	Limits.MaxSpeed = Config->Spacecraft.MaxSpeed;
	const FMissionRunResult Run = MissionRunner::Run(System, State, Replanned.Plan, Limits);
	TestTrue(FString::Printf(TEXT("Replanned playback reaches boundary (failure %d, %.2f AU)"),
		static_cast<int32>(Run.Failure), Run.FinalStarDistance / 1.496e11), Run.bSuccess);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
