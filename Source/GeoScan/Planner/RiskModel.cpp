#include "Planner/RiskModel.h"

namespace RiskModel
{

double EvaluateFlybyRisk(const FRiskModelParams& Params, double PeriapsisRadius,
	double BodyRadius, double Vinf, double FuelSpentFraction)
{
	const double SafeRadius = BodyRadius * Params.SafeAltitudeFactor;

	// 0 at or above the safe radius, 1 at the surface.
	const double ProximityFactor = FMath::Clamp((SafeRadius - PeriapsisRadius) / FMath::Max(SafeRadius - BodyRadius, 1.0), 0.0, 1.0);
	// Faster relative speed: errors in the aim point translate into larger misses.
	const double SpeedFactor = FMath::Clamp(Vinf / FMath::Max(Params.ReferenceVinf, 1.0), 0.0, 1.0);
	// Less remaining fuel: less ability to correct a botched flyby.
	const double FuelFactor = FMath::Clamp(FuelSpentFraction, 0.0, 1.0);

	return FMath::Clamp(
		Params.WeightProximity * ProximityFactor * ProximityFactor +
		Params.WeightSpeed * SpeedFactor * SpeedFactor +
		Params.WeightFuelReserve * FuelFactor,
		0.0, 1.0);
}

} // namespace RiskModel
