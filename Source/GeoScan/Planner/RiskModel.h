#pragma once

#include "CoreMinimal.h"
#include "RiskModel.generated.h"

/**
 * Simplified flyby risk model (task: risk normalized to [0..1], reflecting how
 * close a maneuver is to inadmissible conditions and how sensitive it is to
 * execution errors). All knobs are exposed to the scenario config.
 */
USTRUCT(BlueprintType)
struct FRiskModelParams
{
	GENERATED_BODY()

	/** Risk reaches 0 at periapsis >= BodyRadius * SafeAltitudeFactor. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Risk")
	double SafeAltitudeFactor = 1.5;

	/** Relative speed at which the speed factor saturates at 1, m/s. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Risk")
	double ReferenceVinf = 60000.0;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Risk")
	double WeightProximity = 0.5;

	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Risk")
	double WeightSpeed = 0.3;

	/** Weight of the "no fuel left for trajectory corrections" factor. */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Risk")
	double WeightFuelReserve = 0.2;
};

namespace RiskModel
{
	/**
	 * Periapsis-radius-driven flyby risk, [0..1]. 0 = comfortable pass,
	 * 1 = critical (grazing the surface at high relative speed with no reserve).
	 * Hard violations (periapsis below the surface, etc.) are NOT handled here —
	 * they make the whole edge inadmissible in the planner.
	 */
	double EvaluateFlybyRisk(const FRiskModelParams& Params, double PeriapsisRadius,
		double BodyRadius, double Vinf, double FuelSpentFraction);
}
