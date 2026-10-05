#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "Core/OrbitMath.h"
#include "Gen/SystemGenerator.h"
#include "Sim/MissionPlan.h"
#include "Sim/SpacecraftState.h"
#include "Sim/VisualScale.h"

namespace
{
	bool SimVectorsNearlyEqual(const FVector& A, const FVector& B, double RelativeTolerance)
	{
		const double Scale = FMath::Max(1.0, FMath::Max(A.Size(), B.Size()));
		return (A - B).Size() <= RelativeTolerance * Scale;
	}

	FPlanetarySystem MakeSystem()
	{
		FSystemGenParams Params;
		Params.Seed = 4242;
		Params.NumPlanets = 6;
		return SystemGen::Generate(Params);
	}
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSimInitialStateTest, "GeoScan.Sim.InitialState",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FSimInitialStateTest::RunTest(const FString&)
{
	const FPlanetarySystem System = MakeSystem();

	FSpacecraftFlightParams Params;
	Params.StartingBodyId = FName(TEXT("Planet_2"));
	Params.StartingOrbitRadius = 1.5e8;
	Params.InitialOrbitPhase = 0.7;
	Params.DryMass = 1000.0;
	Params.FuelMass = 4000.0;
	Params.Isp = 3500.0;
	Params.MaxThrust = 250000.0;

	const FSpacecraftState State = SpacecraftSim::MakeInitialState(System, Params);

	TestEqual("Dominant body is the starting planet", State.DominantBodyId, FName(TEXT("Planet_2")));
	TestEqual("Epoch starts at zero", State.Epoch, 0.0);
	TestTrue("Parking orbit radius", FMath::IsNearlyEqual(State.Position.Size(), Params.StartingOrbitRadius, 1.0));

	const FCelestialBodyDesc* Body = System.FindBody(FName(TEXT("Planet_2")));
	const double ExpectedSpeed = OrbitMath::CircularOrbitSpeed(Body->Mu, Params.StartingOrbitRadius);
	TestTrue("Parking orbit speed is circular",
		FMath::IsNearlyEqual(State.Velocity.Size(), ExpectedSpeed, 1e-6 * ExpectedSpeed));

	// Circular orbit: position and velocity stay perpendicular.
	TestTrue("Velocity perpendicular to radius",
		FMath::IsNearlyZero(FVector::DotProduct(State.Position, State.Velocity) / Params.StartingOrbitRadius / ExpectedSpeed, 1e-9));

	TestTrue("Masses copied", State.TotalMass() == 5000.0 && State.FuelMass == 4000.0);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSimSoiSwitchTest, "GeoScan.Sim.SoiSwitch",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FSimSoiSwitchTest::RunTest(const FString&)
{
	const FPlanetarySystem System = MakeSystem();
	const FCelestialBodyDesc* Planet = System.FindBody(FName(TEXT("Planet_3")));
	if (!TestTrue("Planet_3 exists", Planet != nullptr))
	{
		return false;
	}

	// --- Promotion: spacecraft outside the planet's SOI must switch to the star.
	FSpacecraftState Escaping;
	Escaping.DominantBodyId = Planet->Id;
	Escaping.Epoch = 0.0;
	Escaping.Position = FVector(Planet->SoiRadius * 2.0, 0.0, 0.0);
	Escaping.Velocity = FVector(1000.0, 0.0, 0.0);
	Escaping.DryMass = 1000.0;
	Escaping.FuelMass = 1000.0;
	Escaping.Isp = 3000.0;

	FVector WorldPosBefore;
	FVector WorldVelBefore;
	TestTrue("World state before switch", SpacecraftSim::GetWorldState(Escaping, System, 0.0, WorldPosBefore, WorldVelBefore));

	FSpacecraftState Mutated = Escaping;
	SpacecraftSim::UpdateDominantBody(Mutated, System, 0.0);
	TestEqual("Promoted to star", Mutated.DominantBodyId, System.GetStar().Id);

	FVector WorldPosAfter;
	FVector WorldVelAfter;
	SpacecraftSim::GetWorldState(Mutated, System, 0.0, WorldPosAfter, WorldVelAfter);
	TestTrue("Position continuous across switch", SimVectorsNearlyEqual(WorldPosAfter, WorldPosBefore, 1e-9));
	TestTrue("Velocity continuous across switch", SimVectorsNearlyEqual(WorldVelAfter, WorldVelBefore, 1e-9));

	// --- Descent: spacecraft placed inside another planet's SOI switches down to it.
	const FCelestialBodyDesc* Target = System.FindBody(FName(TEXT("Planet_5")));
	FVector TargetPos;
	FVector TargetVel;
	System.GetBodyWorldState(Target->Id, 0.0, TargetPos, TargetVel);

	FSpacecraftState Arriving;
	Arriving.DominantBodyId = System.GetStar().Id;
	Arriving.Epoch = 0.0;
	Arriving.Position = TargetPos + FVector(Target->SoiRadius * 0.5, 0.0, 0.0); // world == heliocentric for the star
	Arriving.Velocity = TargetVel;
	Arriving.DryMass = 1000.0;
	Arriving.FuelMass = 1000.0;
	Arriving.Isp = 3000.0;

	FVector WorldPosBeforeDescent;
	FVector WorldVelBeforeDescent;
	SpacecraftSim::GetWorldState(Arriving, System, 0.0, WorldPosBeforeDescent, WorldVelBeforeDescent);

	SpacecraftSim::UpdateDominantBody(Arriving, System, 0.0);
	TestEqual("Descended into planet SOI", Arriving.DominantBodyId, Target->Id);

	FVector WorldPosDescend;
	FVector WorldVelDescend;
	SpacecraftSim::GetWorldState(Arriving, System, 0.0, WorldPosDescend, WorldVelDescend);
	TestTrue("Position continuous across descent", SimVectorsNearlyEqual(WorldPosDescend, WorldPosBeforeDescent, 1e-9));
	TestTrue("Velocity continuous across descent", SimVectorsNearlyEqual(WorldVelDescend, WorldVelBeforeDescent, 1e-9));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSimImpulseTest, "GeoScan.Sim.Impulse",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FSimImpulseTest::RunTest(const FString&)
{
	const FPlanetarySystem System = MakeSystem();

	FSpacecraftState State = SpacecraftSim::MakeInitialState(System, []{
		FSpacecraftFlightParams Params;
		Params.StartingBodyId = FName(TEXT("Planet_2"));
		Params.DryMass = 1000.0;
		Params.FuelMass = 4000.0;
		Params.Isp = 3500.0;
		return Params;
	}());

	const double BudgetBefore = State.DeltaVRemaining();
	const double FuelBefore = State.FuelMass;
	const double EpochBefore = State.Epoch;

	// Feasible burn: budget drops by exactly the applied delta-v (rocket equation identity).
	TestTrue("Impulse applied", SpacecraftSim::ApplyImpulse(State, System, 600.0, FVector(0.0, 1000.0, 0.0)));
	TestTrue("Budget decreased by the burn",
		FMath::IsNearlyEqual(BudgetBefore - State.DeltaVRemaining(), 1000.0, 1e-6));
	TestTrue("Fuel actually burned", State.FuelMass < FuelBefore);
	TestEqual("Epoch re-snapshotted", State.Epoch, 600.0);
	TestTrue("Epoch advanced from its previous value", EpochBefore < State.Epoch);

	// Infeasible burn: rejected, fuel untouched, velocity untouched.
	const FSpacecraftState Snapshot = State;
	TestFalse("Oversized impulse rejected", SpacecraftSim::ApplyImpulse(State, System, 700.0, FVector(0.0, 1.0e9, 0.0)));
	TestEqual("Fuel unchanged after rejection", State.FuelMass, Snapshot.FuelMass);
	TestTrue("Velocity unchanged after rejection", SimVectorsNearlyEqual(State.Velocity, Snapshot.Velocity, 1e-12));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSimVisualScaleTest, "GeoScan.Sim.VisualScale",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FSimVisualScaleTest::RunTest(const FString&)
{
	FVisualScaleSettings Scale;
	Scale.WorldUnitsPerAU = 2500.0;

	// 1 AU maps to exactly WorldUnitsPerAU units; Z is untouched.
	const FVector Mapped = Scale.ToWorldUnits(FVector(FVisualScaleSettings::AU, 0.0, FVisualScaleSettings::AU));
	TestTrue("1 AU mapping", FMath::IsNearlyEqual(Mapped.X, 2500.0, 1e-6) && FMath::IsNearlyEqual(Mapped.Z, 2500.0, 1e-6));

	// A tiny planet never renders below the minimum visual radius.
	FCelestialBodyDesc TinyPlanet;
	TinyPlanet.ParentId = FName(TEXT("Star"));
	TinyPlanet.Radius = 1.0; // 1 m — sub-pixel at system scale
	TestTrue("Minimum planet visual radius", Scale.BodyVisualRadiusUU(TinyPlanet) == Scale.MinPlanetVisualRadiusUU);

	FCelestialBodyDesc Star;
	Star.ParentId = NAME_None;
	TestTrue("Minimum star visual radius", Scale.BodyVisualRadiusUU(Star) == Scale.MinStarVisualRadiusUU);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSimConstraintsTest, "GeoScan.Sim.Constraints",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FSimConstraintsTest::RunTest(const FString&)
{
	const FPlanetarySystem System = MakeSystem();

	FSpacecraftFlightParams Params;
	Params.StartingBodyId = FName(TEXT("Planet_2"));
	const FSpacecraftState State = SpacecraftSim::MakeInitialState(System, Params);

	FSpacecraftFlightLimits Limits; // everything unlimited
	TestEqual("Clean state passes",
		static_cast<int32>(SpacecraftSim::CheckFlightConstraints(State, System, 1000.0, Limits)),
		static_cast<int32>(EMissionFailure::None));

	Limits.MaxFlightTime = 100.0;
	TestEqual("Flight time limit triggers",
		static_cast<int32>(SpacecraftSim::CheckFlightConstraints(State, System, 1000.0, Limits)),
		static_cast<int32>(EMissionFailure::TimeLimit));

	FSpacecraftFlightLimits SpeedLimits;
	SpeedLimits.MaxSpeed = 1.0; // parking orbit speed is km/s-scale
	TestEqual("Speed limit triggers",
		static_cast<int32>(SpacecraftSim::CheckFlightConstraints(State, System, 0.0, SpeedLimits)),
		static_cast<int32>(EMissionFailure::SpeedLimit));

	FSpacecraftState Crashing = State;
	Crashing.Position = FVector(System.FindBody(FName(TEXT("Planet_2")))->Radius * 0.5, 0.0, 0.0);
	TestEqual("Collision triggers",
		static_cast<int32>(SpacecraftSim::CheckFlightConstraints(Crashing, System, 0.0, FSpacecraftFlightLimits())),
		static_cast<int32>(EMissionFailure::Collision));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSimPlanExecutionTest, "GeoScan.Sim.PlanExecution",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FSimPlanExecutionTest::RunTest(const FString&)
{
	const FPlanetarySystem System = MakeSystem();
	const FCelestialBodyDesc* Body = System.FindBody(FName(TEXT("Planet_2")));
	if (!TestTrue("Planet_2 exists", Body != nullptr))
	{
		return false;
	}

	FSpacecraftState State = SpacecraftSim::MakeInitialState(System, []{
		FSpacecraftFlightParams Params;
		Params.StartingBodyId = FName(TEXT("Planet_2"));
		Params.DryMass = 1000.0;
		Params.FuelMass = 4000.0;
		Params.Isp = 3500.0;
		return Params;
	}());
	const double FuelBefore = State.FuelMass;

	// Escape burn from the circular parking orbit (slightly above the exact value).
	const double ParkingSpeed = OrbitMath::CircularOrbitSpeed(Body->Mu, State.Position.Size());
	const double EscapeDeltaV = ParkingSpeed * (FMath::Sqrt(2.0) - 1.0) * 1.05;
	const FVector Prograde = State.Velocity.GetSafeNormal();

	TArray<FManeuverEvent> Pending;
	Pending.Add({ 1000.0, Prograde * EscapeDeltaV, FName(TEXT("escape")) });

	// Not due yet.
	FPlanExecutionResult Result = SpacecraftSim::ExecuteDueManeuvers(State, System, Pending, 500.0);
	TestEqual("Nothing due at t=500", Result.ExecutedCount, 0);
	TestEqual("Pending still there", Pending.Num(), 1);

	// Due at t=1500 — applied retroactively at its exact event time.
	Result = SpacecraftSim::ExecuteDueManeuvers(State, System, Pending, 1500.0);
	TestEqual("Maneuver executed", Result.ExecutedCount, 1);
	TestEqual("Pending drained", Pending.Num(), 0);
	TestEqual("Failure none", static_cast<int32>(Result.Failure), static_cast<int32>(EMissionFailure::None));
	TestEqual("Applied at event time", State.Epoch, 1000.0);
	TestTrue("Fuel burned", State.FuelMass < FuelBefore);
	TestTrue("Orbit became hyperbolic",
		OrbitMath::SemiMajorAxis(State.Position, State.Velocity, Body->Mu) < 0.0);

	// Unpayable maneuver: rejected with a fuel failure, state untouched.
	FSpacecraftState Fresh = SpacecraftSim::MakeInitialState(System, []{
		FSpacecraftFlightParams Params;
		Params.StartingBodyId = FName(TEXT("Planet_2"));
		Params.DryMass = 1000.0;
		Params.FuelMass = 100.0;
		Params.Isp = 100.0;
		return Params;
	}());
	TArray<FManeuverEvent> AbsurdPending;
	AbsurdPending.Add({ 2000.0, FVector(1.0e9, 0.0, 0.0), FName(TEXT("absurd")) });
	Result = SpacecraftSim::ExecuteDueManeuvers(Fresh, System, AbsurdPending, 3000.0);
	TestEqual("Absurd maneuver rejected", static_cast<int32>(Result.Failure), static_cast<int32>(EMissionFailure::Fuel));
	TestEqual("Nothing executed on rejection", Result.ExecutedCount, 0);
	TestEqual("Fuel untouched on rejection", Fresh.FuelMass, 100.0);

	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
