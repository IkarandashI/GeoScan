#pragma once

#include "CoreMinimal.h"
#include "UObject/Object.h"
#include "RouteCriteria.generated.h"

/** Aggregated route metrics every criterion normalizes into [0..1]. */
USTRUCT(BlueprintType)
struct FRouteMetrics
{
	GENERATED_BODY()

	UPROPERTY(BlueprintReadOnly, Category = "Metrics")
	double TotalDeltaV = 0.0;   // m/s

	UPROPERTY(BlueprintReadOnly, Category = "Metrics")
	double DeltaVBudget = 1.0;  // m/s

	UPROPERTY(BlueprintReadOnly, Category = "Metrics")
	double FlightTime = 0.0;    // s

	UPROPERTY(BlueprintReadOnly, Category = "Metrics")
	double MaxFlightTime = 1.0; // s

	UPROPERTY(BlueprintReadOnly, Category = "Metrics")
	double RiskScore = 0.0;     // [0..1]
};

/**
 * One optimization criterion. The planner only sums weighted criterion scores —
 * new criteria are added as subclasses (or Blueprint children) without touching
 * the planner code (task requirement).
 */
UCLASS(Blueprintable, EditInlineNew, DefaultToInstanced)
class GEOSCAN_API URouteCriterion : public UObject
{
	GENERATED_BODY()

public:
	/** Normalized score in [0..1]; >1 is tolerated and clamped by the caller. */
	UFUNCTION(BlueprintNativeEvent, BlueprintCallable, Category = "Criterion")
	double Evaluate(const FRouteMetrics& Metrics) const;
	virtual double Evaluate_Implementation(const FRouteMetrics& Metrics) const { return 0.0; }

	/** Registry access for config-driven instantiation. */
	static URouteCriterion* CreateById(UObject* Outer, FName CriterionId);
	static bool IsKnownCriterion(FName CriterionId);
};

/** TotalDeltaV / delta-v budget. */
UCLASS(Blueprintable)
class GEOSCAN_API URouteCriterion_Fuel : public URouteCriterion
{
	GENERATED_BODY()

public:
	virtual double Evaluate_Implementation(const FRouteMetrics& Metrics) const override
	{
		return Metrics.TotalDeltaV / FMath::Max(Metrics.DeltaVBudget, 1.0);
	}
};

/** FlightTime / max flight time. */
UCLASS(Blueprintable)
class GEOSCAN_API URouteCriterion_Time : public URouteCriterion
{
	GENERATED_BODY()

public:
	virtual double Evaluate_Implementation(const FRouteMetrics& Metrics) const override
	{
		return Metrics.FlightTime / FMath::Max(Metrics.MaxFlightTime, 1.0);
	}
};

/** Worst-flyby risk as [0..1]. */
UCLASS(Blueprintable)
class GEOSCAN_API URouteCriterion_Risk : public URouteCriterion
{
	GENERATED_BODY()

public:
	virtual double Evaluate_Implementation(const FRouteMetrics& Metrics) const override
	{
		return Metrics.RiskScore;
	}
};
