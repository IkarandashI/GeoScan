#pragma once

#include "CoreMinimal.h"

/** One (seed, mode) benchmark result row. */
struct FBenchmarkRow
{
	int32 Seed = 0;
	FString Mode;
	bool bPlanSuccess = false;
	FString FailReason;
	int32 RouteBodies = 0;
	int32 Maneuvers = 0;
	double TotalDeltaVKmS = 0.0;
	double FlightTimeYears = 0.0;
	double Risk = 0.0;
	double Cost = 0.0;
	double PlanningMs = 0.0;
	bool bPlaybackSuccess = false;
	int32 PlaybackFailure = 0;
	double FuelUsedKg = 0.0;
	double FinalDistanceAU = 0.0;
};

/**
 * Automated benchmark (task bonus): runs planner + playback for every
 * (seed, optimization mode) combination and writes a CSV report.
 */
namespace Benchmark
{
	/** Runs the matrix; when CsvPath is non-empty the report is written there. */
	GEOSCAN_API TArray<FBenchmarkRow> Run(int32 NumSeeds, int32 StartSeed, const FString& CsvPath);
}
