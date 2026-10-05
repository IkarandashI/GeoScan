#include "Config/ScenarioConfig.h"
#include "Planner/RouteCriteria.h"
#include "Sim/SpacecraftState.h"

FSystemGenParams UScenarioConfig::ToGenParams() const
{
	FSystemGenParams Params;
	Params.Seed = static_cast<uint64>(System.Seed);
	Params.NumPlanets = System.NumPlanets;
	Params.StarMass = System.StarMass;
	Params.StarRadius = System.StarRadius;
	Params.MinOrbitalRadius = System.MinOrbitalRadius;
	Params.MaxOrbitalRadius = System.MaxOrbitalRadius;
	Params.BoundaryMode = System.BoundaryMode == EBoundaryMode::FixedDistance
		? ESystemBoundaryMode::FixedDistance
		: ESystemBoundaryMode::RelativeToOutermost;
	Params.FixedBoundaryDistance = System.FixedBoundaryDistance;
	Params.BoundaryK = System.BoundaryK;
	Params.bInclinations = System.bInclinations;
	Params.MaxInclinationDeg = System.MaxInclinationDeg;
	Params.PlanetMassMin = System.PlanetMassMin;
	Params.PlanetMassMax = System.PlanetMassMax;
	Params.PlanetRadiusMin = System.PlanetRadiusMin;
	Params.PlanetRadiusMax = System.PlanetRadiusMax;
	return Params;
}

FSpacecraftFlightParams UScenarioConfig::ToFlightParams() const
{
	FSpacecraftFlightParams Params;
	Params.StartingBodyId = Spacecraft.StartingBodyId;
	Params.StartingOrbitRadius = Spacecraft.StartingOrbitRadius;
	Params.InitialOrbitPhase = Spacecraft.InitialOrbitPhase;
	Params.DryMass = Spacecraft.DryMass;
	Params.FuelMass = Spacecraft.FuelMass;
	Params.MaxThrust = Spacecraft.MaxThrust;
	Params.Isp = Spacecraft.Isp;
	return Params;
}

TArray<FString> UScenarioConfig::Validate(const FPlanetarySystem* GeneratedSystem) const
{
	TArray<FString> Errors;
	auto AddError = [&Errors](FString Message)
	{
		Errors.Add(MoveTemp(Message));
	};

	// --- System generation ---
	if (System.NumPlanets < 1)
	{
		AddError(TEXT("NumPlanets must be at least 1"));
	}
	if (System.StarMass <= 0.0 || System.StarRadius <= 0.0)
	{
		AddError(TEXT("Star mass and radius must be positive"));
	}
	if (System.MinOrbitalRadius <= System.StarRadius * 2.0)
	{
		AddError(TEXT("MinOrbitalRadius is inside the star"));
	}
	if (System.MaxOrbitalRadius <= System.MinOrbitalRadius)
	{
		AddError(TEXT("MaxOrbitalRadius must be greater than MinOrbitalRadius"));
	}
	if (System.BoundaryMode == EBoundaryMode::FixedDistance && System.FixedBoundaryDistance <= System.MaxOrbitalRadius)
	{
		AddError(TEXT("Fixed boundary distance must lie beyond the outermost possible orbit"));
	}
	if (System.BoundaryMode == EBoundaryMode::RelativeToOutermost && System.BoundaryK <= 1.0)
	{
		AddError(TEXT("BoundaryK must be greater than 1"));
	}
	if (System.PlanetMassMin <= 0.0 || System.PlanetMassMax < System.PlanetMassMin
		|| System.PlanetRadiusMin <= 0.0 || System.PlanetRadiusMax < System.PlanetRadiusMin)
	{
		AddError(TEXT("Planet mass/radius ranges are invalid"));
	}

	// --- Spacecraft ---
	if (Spacecraft.DryMass <= 0.0 || Spacecraft.Isp <= 0.0 || Spacecraft.MaxThrust <= 0.0)
	{
		AddError(TEXT("DryMass, Isp and MaxThrust must be positive"));
	}
	if (Spacecraft.FuelMass < 0.0)
	{
		AddError(TEXT("FuelMass must not be negative"));
	}
	if (Spacecraft.StartingOrbitRadius <= 0.0)
	{
		AddError(TEXT("StartingOrbitRadius must be positive"));
	}
	if (Spacecraft.MaxFlightTime <= 0.0)
	{
		AddError(TEXT("MaxFlightTime must be positive"));
	}

	// --- Optimization ---
	if (Optimization.Criteria.IsEmpty())
	{
		AddError(TEXT("At least one optimization criterion with a weight is required"));
	}
	else
	{
		double WeightSum = 0.0;
		for (const FCriterionWeight& Criterion : Optimization.Criteria)
		{
			if (!URouteCriterion::IsKnownCriterion(Criterion.CriterionId))
			{
				AddError(FString::Printf(TEXT("Unknown criterion id '%s'"),
					*Criterion.CriterionId.ToString()));
			}
			WeightSum += FMath::Abs(Criterion.Weight);
		}
		if (WeightSum <= 0.0)
		{
			AddError(TEXT("Sum of criterion weights must be positive"));
		}
	}

	// --- Against the generated system ---
	if (GeneratedSystem != nullptr)
	{
		const FCelestialBodyDesc* StartingBody = GeneratedSystem->FindBody(Spacecraft.StartingBodyId);
		if (StartingBody == nullptr)
		{
			AddError(FString::Printf(TEXT("Starting body '%s' not found in the generated system"),
				*Spacecraft.StartingBodyId.ToString()));
		}
		else
		{
			if (StartingBody->ParentId == NAME_None)
			{
				AddError(TEXT("Starting body must be a planet, not the star"));
			}
			else if (StartingBody->OrbitRadius >= GeneratedSystem->OutermostChildOrbitRadius() - 1.0)
			{
				AddError(TEXT("Starting body must not be the outermost body of the system (task requirement)"));
			}
			if (Spacecraft.StartingOrbitRadius <= StartingBody->Radius)
			{
				AddError(TEXT("Starting orbit radius is below the surface of the starting body"));
			}
			if (StartingBody->SoiRadius > 0.0 && Spacecraft.StartingOrbitRadius >= StartingBody->SoiRadius)
			{
				AddError(TEXT("Starting orbit radius lies outside the sphere of influence of the starting body"));
			}
		}
	}

	return Errors;
}

void UScenarioConfig::SyncWeightsToMode()
{
	Optimization.Criteria.Reset();

	auto Add = [this](FName Id, double Weight)
	{
		FCriterionWeight& Criterion = Optimization.Criteria.AddDefaulted_GetRef();
		Criterion.CriterionId = Id;
		Criterion.Weight = Weight;
	};

	switch (Optimization.Mode)
	{
	case EOptimizationMode::FuelEfficient:
		Add(TEXT("Fuel"), 1.0);
		Add(TEXT("Time"), 0.0);
		Add(TEXT("Risk"), 0.0);
		break;
	case EOptimizationMode::TimeEfficient:
		Add(TEXT("Fuel"), 0.0);
		Add(TEXT("Time"), 1.0);
		Add(TEXT("Risk"), 0.0);
		break;
	case EOptimizationMode::Balanced:
	default:
		Add(TEXT("Fuel"), 0.6);
		Add(TEXT("Time"), 0.3);
		Add(TEXT("Risk"), 0.1);
		break;
	}
}

void UScenarioConfig::ApplyPreset(EOptimizationMode Mode)
{
	System = FSystemGenerationSetup();
	Spacecraft = FSpacecraftSetup();
	Optimization = FOptimizationSetup();
	Optimization.Mode = Mode;
	SyncWeightsToMode();

	// Fuel-optimal escapes coast out on slow ellipses; give them the room they need.
	Spacecraft.MaxFlightTime = Mode == EOptimizationMode::FuelEfficient ? 3.156e10   // ~1000 years
		: Mode == EOptimizationMode::TimeEfficient ? 1.578e9                          // ~50 years
		: 3.156e9;                                                                     // ~100 years
}
