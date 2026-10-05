#include "Systems/BenchmarkRunner.h"

#include "Config/ScenarioConfig.h"
#include "Gen/SystemGenerator.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Planner/ScenarioPlanner.h"
#include "Sim/MissionRunner.h"
#include "Sim/SpacecraftState.h"
#include "UObject/Package.h"

namespace
{

FString ModeToString(EOptimizationMode Mode)
{
	switch (Mode)
	{
	case EOptimizationMode::FuelEfficient: return TEXT("Fuel");
	case EOptimizationMode::TimeEfficient: return TEXT("Time");
	default: return TEXT("Balanced");
	}
}

}

namespace Benchmark
{

TArray<FBenchmarkRow> Run(int32 NumSeeds, int32 StartSeed, const FString& CsvPath)
{
	TArray<FBenchmarkRow> Rows;

	for (int32 Seed = StartSeed; Seed < StartSeed + NumSeeds; ++Seed)
	{
		for (const EOptimizationMode Mode : { EOptimizationMode::FuelEfficient, EOptimizationMode::TimeEfficient, EOptimizationMode::Balanced })
		{
			UScenarioConfig* Config = NewObject<UScenarioConfig>(GetTransientPackage());
			Config->ApplyPreset(Mode);
			Config->System.Seed = Seed;

			FBenchmarkRow Row;
			Row.Seed = Seed;
			Row.Mode = ModeToString(Mode);

			const FPlanetarySystem System = SystemGen::Generate(Config->ToGenParams());
			const TArray<FString> Errors = Config->Validate(&System);
			if (!Errors.IsEmpty())
			{
				Row.FailReason = Errors[0];
				Rows.Add(Row);
				continue;
			}

			const FSpacecraftState Initial = SpacecraftSim::MakeInitialState(System, Config->ToFlightParams());

			const double StartSeconds = FPlatformTime::Seconds();
			const FPlanningResult Planned = ScenarioPlanner::PlanScenario(Config, System, Initial);
			Row.PlanningMs = (FPlatformTime::Seconds() - StartSeconds) * 1000.0;

			Row.bPlanSuccess = Planned.bSuccess;
			Row.FailReason = Planned.FailReason;
			Row.RouteBodies = Planned.RouteBodies.Num();
			Row.Maneuvers = Planned.Plan.Maneuvers.Num();
			Row.TotalDeltaVKmS = Planned.TotalDeltaV / 1000.0;
			Row.FlightTimeYears = Planned.FlightTime / 31557600.0;
			Row.Risk = Planned.RiskScore;
			Row.Cost = Planned.Cost;

			if (Planned.bSuccess)
			{
				FSpacecraftFlightLimits Limits;
				Limits.MaxFlightTime = Config->Spacecraft.MaxFlightTime;
				Limits.MaxSpeed = Config->Spacecraft.MaxSpeed;
				const FMissionRunResult Playback = MissionRunner::Run(System, Initial, Planned.Plan, Limits);
				Row.bPlaybackSuccess = Playback.bSuccess;
				Row.PlaybackFailure = static_cast<int32>(Playback.Failure);
				Row.FuelUsedKg = Playback.FuelUsed;
				Row.FinalDistanceAU = Playback.FinalStarDistance / 1.496e11;
			}
			if (!Row.bPlaybackSuccess)
			{
				UE_LOG(LogTemp, Warning, TEXT("[Benchmark] seed %d %s FAILED: failure=%d final=%.2f AU, planned dV=%.2f km/s t=%.2f y"),
					Seed, *Row.Mode, Row.PlaybackFailure, Row.FinalDistanceAU, Row.TotalDeltaVKmS, Row.FlightTimeYears);
			}
			Rows.Add(Row);
		}
	}

	if (!CsvPath.IsEmpty() && Rows.Num() > 0)
	{
		IFileManager::Get().MakeDirectory(*FPaths::GetPath(CsvPath), true);

		FString Csv = TEXT("Seed,Mode,PlanSuccess,RouteBodies,Maneuvers,DeltaV_km_s,FlightTime_years,Risk,Cost,PlanMs,PlaybackSuccess,PlaybackFailure,FuelUsed_kg\r\n");
		for (const FBenchmarkRow& Row : Rows)
		{
			Csv += FString::Printf(TEXT("%d,%s,%s,%d,%d,%.3f,%.3f,%.3f,%.4f,%.1f,%s,%d,%.1f\r\n"),
				Row.Seed, *Row.Mode, Row.bPlanSuccess ? TEXT("1") : TEXT("0"),
				Row.RouteBodies, Row.Maneuvers, Row.TotalDeltaVKmS, Row.FlightTimeYears,
				Row.Risk, Row.Cost, Row.PlanningMs,
				Row.bPlaybackSuccess ? TEXT("1") : TEXT("0"), Row.PlaybackFailure, Row.FuelUsedKg);
		}
		FFileHelper::SaveStringToFile(Csv, *CsvPath, FFileHelper::EEncodingOptions::ForceUTF8WithoutBOM);
	}

	return Rows;
}

} // namespace Benchmark
