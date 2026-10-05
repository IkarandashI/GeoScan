#pragma once

#include "CoreMinimal.h"
#include "Engine/DataAsset.h"
#include "Gen/SystemGenerator.h"
#include "Planner/RiskModel.h"
#include "Sim/SpacecraftState.h"
#include "ScenarioConfig.generated.h"

UENUM(BlueprintType)
enum class EBoundaryMode : uint8
{
	FixedDistance UMETA(DisplayName = "Fixed Distance"),
	RelativeToOutermost UMETA(DisplayName = "K * Outermost Orbit")
};

UENUM(BlueprintType)
enum class EOptimizationMode : uint8
{
	FuelEfficient,
	TimeEfficient,
	Balanced
};

/** One entry of the weighted cost sum; the planner only ever sees this list. */
USTRUCT(BlueprintType)
struct FCriterionWeight
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	FName CriterionId;

	UPROPERTY(EditAnywhere, BlueprintReadWrite)
	double Weight = 0.0;
};

USTRUCT(BlueprintType)
struct FSystemGenerationSetup
{
	GENERATED_BODY()

	// int64 instead of uint64: Blueprint serialization does not support uint64.
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "System")
	int64 Seed = 12345;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "System", meta = (ClampMin = "1", ClampMax = "24"))
	int32 NumPlanets = 6;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "System")
	double StarMass = 1.989e30;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "System")
	double StarRadius = 6.96e8;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "System", meta = (ClampMin = "0"))
	double MinOrbitalRadius = 6.0e10;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "System", meta = (ClampMin = "0"))
	double MaxOrbitalRadius = 7.5e12;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "System")
	EBoundaryMode BoundaryMode = EBoundaryMode::FixedDistance;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "System", meta = (ClampMin = "0"))
	double FixedBoundaryDistance = 1.8e13;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "System", meta = (ClampMin = "1"))
	double BoundaryK = 1.5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "System")
	bool bInclinations = false;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "System", meta = (ClampMin = "0", ClampMax = "90"))
	double MaxInclinationDeg = 8.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "System|Planets")
	double PlanetMassMin = 1.0e24;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "System|Planets")
	double PlanetMassMax = 1.9e27;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "System|Planets")
	double PlanetRadiusMin = 2.4e6;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "System|Planets")
	double PlanetRadiusMax = 5.0e7;
};

USTRUCT(BlueprintType)
struct FSpacecraftSetup
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spacecraft", meta = (ClampMin = "0.1"))
	double DryMass = 1000.0;        // kg

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spacecraft", meta = (ClampMin = "0"))
	double FuelMass = 4000.0;       // kg

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spacecraft", meta = (ClampMin = "0"))
	double MaxThrust = 250000.0;    // N

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spacecraft", meta = (ClampMin = "0.1"))
	double Isp = 3500.0;            // s

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spacecraft")
	FName StartingBodyId = FName("Planet_2");

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spacecraft", meta = (ClampMin = "0"))
	double StartingOrbitRadius = 1.5e8; // m

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spacecraft")
	double InitialOrbitPhase = 0.0;     // rad

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spacecraft")
	double MaxFlightTime = 3.156e9;      // s (~100 years)

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Spacecraft")
	double MaxSpeed = 2.5e5;            // m/s, hard limit; <= 0 = unlimited
};

USTRUCT(BlueprintType)
struct FOptimizationSetup
{
	GENERATED_BODY()

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimization")
	EOptimizationMode Mode = EOptimizationMode::Balanced;

	/** Weighted cost = Sum(Criterion.Weight * Criterion[0..1] score). */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimization")
	TArray<FCriterionWeight> Criteria;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimization|Search", meta = (ClampMin = "1", ClampMax = "32"))
	int32 BeamWidth = 6;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimization|Search", meta = (ClampMin = "1", ClampMax = "8"))
	int32 MaxHops = 4;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimization|Search", meta = (ClampMin = "1"))
	int32 DepartureWindowSteps = 6;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimization|Search", meta = (ClampMin = "1"))
	int32 FlightTimeSteps = 6;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimization|Search", meta = (ClampMin = "1"))
	double MinLegFlightDays = 20.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimization|Search", meta = (ClampMin = "2"))
	double MaxLegFlightDays = 4000.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimization|Search", meta = (ClampMin = "0"))
	double MaxDepartureWaitDays = 600.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimization|Search", meta = (ClampMin = "1.05", ClampMax = "5"))
	double PeriapsisSafetyFactor = 1.5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Optimization")
	FRiskModelParams RiskParams;
};

class ACelestialBodyActor;
class ASpacecraftActor;

/**
 * Scenario configurator: one DataAsset = one mission scenario.
 * Simulation and planner consume these values only — nothing about the mission
 * is hardcoded (test task requirement).
 */
UCLASS(BlueprintType)
class GEOSCAN_API UScenarioConfig : public UPrimaryDataAsset
{
	GENERATED_BODY()

public:
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
	FSystemGenerationSetup System;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
	FSpacecraftSetup Spacecraft;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario")
	FOptimizationSetup Optimization;

	/** Visual classes: point these at Blueprint subclasses to restyle bodies
	 *  without touching code; empty = built-in C++ actors. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Visuals")
	TSoftClassPtr<ACelestialBodyActor> StarClass;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Visuals")
	TSoftClassPtr<ACelestialBodyActor> PlanetClass;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Scenario|Visuals")
	TSoftClassPtr<ASpacecraftActor> SpacecraftClass;

	/** Domain-level generation parameters for SystemGen::Generate. */
	FSystemGenParams ToGenParams() const;

	/** Domain-level spacecraft parameters for SpacecraftSim::MakeInitialState. */
	FSpacecraftFlightParams ToFlightParams() const;

	/**
	 * Config-level checks always run; when a generated system is passed in,
	 * the starting body is validated against it (exists, not the outermost body,
	 * starting orbit above the surface and inside the SOI).
	 */
	TArray<FString> Validate(const FPlanetarySystem* GeneratedSystem = nullptr) const;

	/** Replaces the criteria weights with the preset of the current Mode. */
	UFUNCTION(BlueprintCallable, Category = "Scenario")
	void SyncWeightsToMode();

	/** Fills the whole asset with defaults plus the given mode preset. */
	UFUNCTION(BlueprintCallable, Category = "Scenario")
	void ApplyPreset(EOptimizationMode Mode);
};
