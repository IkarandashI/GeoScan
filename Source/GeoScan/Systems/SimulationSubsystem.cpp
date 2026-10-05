#include "Systems/SimulationSubsystem.h"

#include "Actors/CelestialBodyActor.h"
#include "Actors/SpacecraftActor.h"
#include "Components/LineBatchComponent.h"
#include "Config/ScenarioConfig.h"
#include "Config/ScenarioJson.h"
#include "Core/OrbitMath.h"
#include "Engine/World.h"
#include "Gen/SystemGenerator.h"
#include "HAL/FileManager.h"
#include "HAL/IConsoleManager.h"
#include "Misc/DateTime.h"
#include "Misc/Paths.h"
#include "Planner/RoutePlanner.h"
#include "Planner/ScenarioPlanner.h"
#include "Sim/MissionRunner.h"
#include "Systems/BenchmarkRunner.h"

DEFINE_LOG_CATEGORY_STATIC(LogGeoScan, Log, All);

void USimulationSubsystem::Initialize(FSubsystemCollectionBase& Collection)
{
	Super::Initialize(Collection);
}

void USimulationSubsystem::Deinitialize()
{
	Super::Deinitialize();
}

void USimulationSubsystem::Tick(float DeltaTime)
{
	if (Status != EMissionStatus::Running || TimeWarp <= 0.0)
	{
		return;
	}

	SimTime += static_cast<double>(DeltaTime) * TimeWarp;

	// Autopilot: apply due maneuvers at their exact event times (warp-safe).
	if (!PendingManeuvers.IsEmpty())
	{
		const FPlanExecutionResult Execution = SpacecraftSim::ExecuteDueManeuvers(Spacecraft, System, PendingManeuvers, SimTime);
		ExecutedManeuvers += Execution.ExecutedCount;
		if (Execution.ExecutedCount > 0)
		{
			UE_LOG(LogGeoScan, Log, TEXT("Executed %d maneuver(s) by t=%.1f d; %d pending, fuel %.1f kg"),
				Execution.ExecutedCount, SimTime / 86400.0, PendingManeuvers.Num(), Spacecraft.FuelMass);
		}
		if (Execution.Failure == EMissionFailure::Fuel)
		{
			FinishMission(EMissionStatus::FailedFuel, TEXT("A planned maneuver needs more fuel than available"));
			return;
		}
	}

	SpacecraftSim::UpdateDominantBody(Spacecraft, System, SimTime);

	// Success condition (task): reaching the system boundary distance from the star.
	FVector Position;
	FVector Velocity;
	if (GetSpacecraftWorldState(Position, Velocity) && Position.Size() >= System.SystemBoundary)
	{
		FinishMission(EMissionStatus::Success,
			FString::Printf(TEXT("Boundary reached after %.2f years"), SimTime / 31557600.0));
		return;
	}

	// Hard failure conditions (task): collision, speed limit, flight time limit.
	switch (SpacecraftSim::CheckFlightConstraints(Spacecraft, System, SimTime, FlightLimits))
	{
	case EMissionFailure::Collision:
		FinishMission(EMissionStatus::FailedCollision, TEXT("Spacecraft collided with a celestial body"));
		return;
	case EMissionFailure::SpeedLimit:
		FinishMission(EMissionStatus::FailedSpeed, TEXT("Maximum speed exceeded"));
		return;
	case EMissionFailure::TimeLimit:
		FinishMission(EMissionStatus::FailedTime, TEXT("Maximum flight time exceeded"));
		return;
	default:
		break;
	}

	// Keep the debug overlay in step with executed maneuvers / SOI switches.
	if (bDebugVisualization
		&& (Spacecraft.Epoch != LastVizEpoch || PendingManeuvers.Num() != LastVizPendingCount))
	{
		UpdateDebugVisualization();
	}
}

void USimulationSubsystem::SetDebugVisualization(bool bEnabled)
{
	bDebugVisualization = bEnabled;
	if (bEnabled)
	{
		UpdateDebugVisualization();
	}
	else if (ULineBatchComponent* Batch = GetDebugBatcher())
	{
		Batch->Flush();
	}
}

ULineBatchComponent* USimulationSubsystem::GetDebugBatcher()
{
	if (DebugBatcher.IsValid())
	{
		return DebugBatcher.Get();
	}
	UWorld* World = GetWorld();
	if (World == nullptr)
	{
		return nullptr;
	}
	AActor* Host = World->SpawnActor<AActor>();
	DebugBatchHost = Host;
	DebugBatcher = NewObject<ULineBatchComponent>(Host);
	DebugBatcher->RegisterComponent();
	return DebugBatcher.Get();
}

void USimulationSubsystem::UpdateDebugVisualization()
{
	if (!bDebugVisualization || Status == EMissionStatus::Idle)
	{
		return;
	}
	ULineBatchComponent* Batch = GetDebugBatcher();
	if (Batch == nullptr)
	{
		return;
	}
	Batch->Flush();

	const FLinearColor SoiColor(0.35f, 0.2f, 0.55f, 1.0f);
	const FLinearColor RouteColor(0.2f, 0.85f, 0.3f, 1.0f);
	const FLinearColor ManeuverColor(1.0f, 0.85f, 0.2f, 1.0f);

	auto DrawRing = [Batch](const FVector& CenterUU, double RadiusUU, const FVector& AxisA, const FVector& AxisB, const FLinearColor& Color)
	{
		constexpr int32 Segments = 48;
		FVector Previous;
		for (int32 Index = 0; Index <= Segments; ++Index)
		{
			const double Theta = 2.0 * UE_DOUBLE_PI * Index / Segments;
			const FVector Point = CenterUU + (FMath::Cos(Theta) * AxisA + FMath::Sin(Theta) * AxisB) * RadiusUU;
			if (Index > 0)
			{
				Batch->DrawLine(Previous, Point, Color, 1.0f, 0.0f);
			}
			Previous = Point;
		}
	};

	// SOI rings around every planet (two planes for a 3D read).
	for (const FCelestialBodyDesc& Body : System.Bodies)
	{
		if (Body.ParentId == NAME_None || Body.SoiRadius <= 0.0)
		{
			continue;
		}
		FVector Position;
		FVector Velocity;
		System.GetBodyWorldState(Body.Id, SimTime, Position, Velocity);
		const FVector CenterUU = VisualScale.ToWorldUnits(Position);
		const double RadiusUU = VisualScale.VisualSoiRadiusUU(Body);
		DrawRing(CenterUU, RadiusUU, FVector::XAxisVector, FVector::YAxisVector, SoiColor);
		DrawRing(CenterUU, RadiusUU, FVector::XAxisVector, FVector::ZAxisVector, SoiColor);
	}

	// Remaining planned route, played forward through the same physics the mission uses.
	FMissionPlan Remaining;
	Remaining.Maneuvers = PendingManeuvers;
	TArray<FVector> Path;
	FSpacecraftFlightLimits PlaybackLimits = FlightLimits;
	if (PlaybackLimits.MaxFlightTime <= 0.0)
	{
		PlaybackLimits.MaxFlightTime = SimTime + 100.0 * 31557600.0;
	}
	MissionRunner::Run(System, Spacecraft, Remaining, PlaybackLimits, &Path);
	if (Path.Num() >= 2)
	{
		const int32 Stride = FMath::Max(1, Path.Num() / 1200);
		FVector PreviousUU = VisualScale.ToWorldUnits(Path[0]);
		for (int32 Index = Stride; Index < Path.Num(); Index += Stride)
		{
			const FVector PointUU = VisualScale.ToWorldUnits(Path[Index]);
			Batch->DrawLine(PreviousUU, PointUU, RouteColor, 2.0f, 0.0f);
			PreviousUU = PointUU;
		}
	}

	// Crosshair markers at every pending maneuver point.
	const FCelestialBodyDesc* Dominant = System.FindBody(Spacecraft.DominantBodyId);
	if (Dominant)
	{
		for (const FManeuverEvent& Maneuver : PendingManeuvers)
		{
			FVector RelativePosition;
			FVector RelativeVelocity;
			OrbitMath::Propagate(Spacecraft.Position, Spacecraft.Velocity, Dominant->Mu,
				Maneuver.Time - Spacecraft.Epoch, RelativePosition, RelativeVelocity);
			FVector BodyPosition;
			FVector BodyVelocity;
			System.GetBodyWorldState(Dominant->Id, Maneuver.Time, BodyPosition, BodyVelocity);
			const FVector PointUU = VisualScale.ToVisualAroundBody(*Dominant, BodyPosition, BodyPosition + RelativePosition);
			const double Size = 12.0;
			Batch->DrawLine(PointUU - FVector(Size, 0, 0), PointUU + FVector(Size, 0, 0), ManeuverColor, 2.0f, 0.0f);
			Batch->DrawLine(PointUU - FVector(0, Size, 0), PointUU + FVector(0, Size, 0), ManeuverColor, 2.0f, 0.0f);
			Batch->DrawLine(PointUU - FVector(0, 0, Size), PointUU + FVector(0, 0, Size), ManeuverColor, 2.0f, 0.0f);
		}
	}

	LastVizEpoch = Spacecraft.Epoch;
	LastVizPendingCount = PendingManeuvers.Num();
}

void USimulationSubsystem::FinishMission(EMissionStatus InStatus, const FString& Reason)
{
	Status = InStatus;
	TimeWarp = 0.0;
	if (InStatus != EMissionStatus::Success)
	{
		FailureReason = Reason;
	}
	UE_LOG(LogGeoScan, Log, TEXT("Mission finished: %s (%s) at t=%.1f d"),
		*UEnum::GetValueAsString(InStatus), *Reason, SimTime / 86400.0);
}

TStatId USimulationSubsystem::GetStatId() const
{
	RETURN_QUICK_DECLARE_CYCLE_STAT(USimulationSubsystem, STATGROUP_Tickables);
}

bool USimulationSubsystem::StartScenario(UScenarioConfig* Config)
{
	if (Config == nullptr)
	{
		UE_LOG(LogGeoScan, Error, TEXT("StartScenario: config is null"));
		return false;
	}

	System = SystemGen::Generate(Config->ToGenParams());

	const TArray<FString> Errors = Config->Validate(&System);
	if (!Errors.IsEmpty())
	{
		for (const FString& Error : Errors)
		{
			UE_LOG(LogGeoScan, Error, TEXT("Scenario invalid: %s"), *Error);
		}
		return false;
	}

	Scenario = Config;
	Spacecraft = SpacecraftSim::MakeInitialState(System, Config->ToFlightParams());
	SimTime = 0.0;
	SetWarpIndex(5); // live from the start: 10 days/sec — orbital motion is visible
	Status = EMissionStatus::Running;
	FailureReason.Reset();
	FlightLimits.MaxFlightTime = Config->Spacecraft.MaxFlightTime;
	FlightLimits.MaxSpeed = Config->Spacecraft.MaxSpeed;
	ClearPlan();

	SpawnActors();

	UE_LOG(LogGeoScan, Log, TEXT("Scenario '%s' started: %d bodies, boundary %.2f AU, start at '%s' (r=%.1e m), fuel %.0f kg, dv budget %.2f km/s"),
		*Config->GetName(), System.Bodies.Num(), System.SystemBoundary / FVisualScaleSettings::AU,
		*Spacecraft.DominantBodyId.ToString(), Config->Spacecraft.StartingOrbitRadius,
		Spacecraft.FuelMass, Spacecraft.DeltaVRemaining() / 1000.0);

	// The mission is autonomous end-to-end: plan the route right away (task: the
	// planning system picks the maneuver sequence by the chosen criterion).
	PlanMission();
	return true;
}

bool USimulationSubsystem::GetSpacecraftWorldState(FVector& OutPosition, FVector& OutVelocity) const
{
	if (Status == EMissionStatus::Idle)
	{
		return false;
	}
	return SpacecraftSim::GetWorldState(Spacecraft, System, SimTime, OutPosition, OutVelocity);
}

bool USimulationSubsystem::GetSpacecraftVisualPosition(FVector& OutPositionUU) const
{
	FVector WorldPosition;
	FVector WorldVelocity;
	if (!GetSpacecraftWorldState(WorldPosition, WorldVelocity))
	{
		return false;
	}
	if (const FCelestialBodyDesc* Dominant = System.FindBody(Spacecraft.DominantBodyId))
	{
		FVector BodyPosition;
		FVector BodyVelocity;
		System.GetBodyWorldState(Dominant->Id, SimTime, BodyPosition, BodyVelocity);
		OutPositionUU = VisualScale.ToVisualAroundBody(*Dominant, BodyPosition, WorldPosition);
	}
	else
	{
		OutPositionUU = VisualScale.ToWorldUnits(WorldPosition);
	}
	return true;
}

const TArray<double>& USimulationSubsystem::GetWarpLevels()
{
	// 0 = paused, then: real-time, minute, hour, day, 10 days, month, year, decade.
	static const TArray<double> Levels = { 0.0, 1.0, 60.0, 3600.0, 86400.0, 864000.0, 2629800.0, 31557600.0, 315576000.0 };
	return Levels;
}

int32 USimulationSubsystem::GetWarpIndex() const
{
	const TArray<double>& Levels = GetWarpLevels();
	for (int32 Index = 0; Index < Levels.Num(); ++Index)
	{
		if (FMath::IsNearlyEqual(Levels[Index], TimeWarp))
		{
			return Index;
		}
	}
	return 0;
}

void USimulationSubsystem::SetWarpIndex(int32 Index)
{
	const TArray<double>& Levels = GetWarpLevels();
	TimeWarp = Levels[FMath::Clamp(Index, 0, Levels.Num() - 1)];
}

void USimulationSubsystem::LoadPlan(const FMissionPlan& Plan)
{
	LoadedPlan = Plan;
	PendingManeuvers = Plan.Maneuvers;
	PendingManeuvers.Sort([](const FManeuverEvent& A, const FManeuverEvent& B) { return A.Time < B.Time; });
	ExecutedManeuvers = 0;
	UE_LOG(LogGeoScan, Log, TEXT("Plan loaded: %d maneuvers, total dV %.2f km/s"),
		PendingManeuvers.Num(), Plan.TotalDeltaV / 1000.0);
}

bool USimulationSubsystem::PlanMission()
{
	if (Scenario == nullptr)
	{
		return false;
	}
	const double StartSeconds = FPlatformTime::Seconds();
	const FPlanningResult Result = ScenarioPlanner::PlanScenario(Scenario, System, Spacecraft);
	const double ElapsedMs = (FPlatformTime::Seconds() - StartSeconds) * 1000.0;

	if (!Result.bSuccess)
	{
		UE_LOG(LogGeoScan, Error, TEXT("Planning failed: %s"), *Result.FailReason);
		return false;
	}

	LoadPlan(Result.Plan);
	RouteBodies = Result.RouteBodies;

	FString RouteText;
	for (const FName& Body : Result.RouteBodies)
	{
		RouteText += FString::Printf(TEXT("%s -> "), *Body.ToString());
	}
	UE_LOG(LogGeoScan, Log,
		TEXT("Planned in %.1f ms: route %sboundary | dV %.2f km/s, %.2f years, risk %.3f, cost %.3f (%d maneuvers)"),
		ElapsedMs, *RouteText, Result.TotalDeltaV / 1000.0, Result.FlightTime / 31557600.0,
		Result.RiskScore, Result.Cost, Result.Plan.Maneuvers.Num());
	return true;
}

bool USimulationSubsystem::ReplanMission()
{
	if (Status != EMissionStatus::Running || Scenario == nullptr)
	{
		return false;
	}

	// Re-snapshot the spacecraft at the current sim time.
	FSpacecraftState Current = Spacecraft;
	if (const FCelestialBodyDesc* Dominant = System.FindBody(Current.DominantBodyId))
	{
		OrbitMath::Propagate(Current.Position, Current.Velocity, Dominant->Mu,
			SimTime - Current.Epoch, Current.Position, Current.Velocity);
		Current.Epoch = SimTime;
	}

	FPlanRequest Request = ScenarioPlanner::MakeRequest(Scenario, System, Current);
	const FPlanningResult Result = RoutePlanner::PlanRoute(Request);
	if (!Result.bSuccess)
	{
		UE_LOG(LogGeoScan, Warning, TEXT("Replanning failed: %s"), *Result.FailReason);
		return false;
	}

	LoadPlan(Result.Plan);
	RouteBodies = Result.RouteBodies;
	UE_LOG(LogGeoScan, Log, TEXT("Replanned at t=%.1f d: %d maneuvers, dV %.2f km/s, %.2f years left"),
		SimTime / 86400.0, Result.Plan.Maneuvers.Num(), Result.TotalDeltaV / 1000.0,
		(FMath::Max(0.0, FlightLimits.MaxFlightTime - SimTime)) / 31557600.0);
	return true;
}

void USimulationSubsystem::ClearPlan()
{
	LoadedPlan = FMissionPlan();
	PendingManeuvers.Reset();
	RouteBodies.Reset();
	ExecutedManeuvers = 0;
}

bool USimulationSubsystem::GetNextManeuver(FManeuverEvent& OutEvent) const
{
	if (PendingManeuvers.IsEmpty())
	{
		return false;
	}
	OutEvent = PendingManeuvers[0];
	return true;
}

void USimulationSubsystem::SpawnActors()
{
	UWorld* World = GetWorld();
	BodyActors.Reset();

	// Visual classes may be Blueprint subclasses set on the scenario; empty = C++ actors.
	// Note: LoadSynchronous on an unset soft class simply returns null — IsValid()
	// would be wrong here (it checks whether the object is already loaded).
	UClass* StarClass = Scenario ? Scenario->StarClass.LoadSynchronous() : nullptr;
	UClass* PlanetClass = Scenario ? Scenario->PlanetClass.LoadSynchronous() : nullptr;

	for (const FCelestialBodyDesc& Body : System.Bodies)
	{
		UClass* ActorClass = Body.ParentId == NAME_None
			? (StarClass ? StarClass : ACelestialBodyActor::StaticClass())
			: (PlanetClass ? PlanetClass : ACelestialBodyActor::StaticClass());

		ACelestialBodyActor* Actor = World->SpawnActor<ACelestialBodyActor>(ActorClass, FVector::ZeroVector, FRotator::ZeroRotator);
		if (Actor)
		{
			Actor->Init(this, Body);
			BodyActors.Add(Actor);
		}
	}

	UClass* SpacecraftClass = Scenario ? Scenario->SpacecraftClass.LoadSynchronous() : nullptr;
	SpacecraftActor = World->SpawnActor<ASpacecraftActor>(
		SpacecraftClass ? SpacecraftClass : ASpacecraftActor::StaticClass(),
		FVector::ZeroVector, FRotator::ZeroRotator);
	if (SpacecraftActor.IsValid())
	{
		SpacecraftActor->Init(this);
	}
}

// ---------------------------------------------------------------- Console commands

namespace
{
	void ParseConsoleArgs(const TArray<FString>& Args, TMap<FString, FString>& Out)
	{
		for (const FString& Arg : Args)
		{
			FString Key;
			FString Value;
			if (Arg.Split(TEXT("="), &Key, &Value))
			{
				Out.Add(Key, Value);
			}
		}
	}

	USimulationSubsystem* GetSim(UWorld* World)
	{
		return World ? World->GetSubsystem<USimulationSubsystem>() : nullptr;
	}
}

static FAutoConsoleCommandWithWorldAndArgs GeoScanBenchmarkCmd(
	TEXT("GeoScan.Benchmark"),
	TEXT("Runs the planner+playback benchmark. Args: Seeds=N StartSeed=S. Writes CSV to Saved/Benchmarks."),
	FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
	{
		TMap<FString, FString> Parsed;
		ParseConsoleArgs(Args, Parsed);
		const int32 NumSeeds = FMath::Clamp(FCString::Atoi(*Parsed.FindRef(TEXT("Seeds"))), 1, 100);
		const int32 StartSeed = FCString::Atoi(*Parsed.FindRef(TEXT("StartSeed")));

		const FString CsvPath = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Benchmarks"),
			FString::Printf(TEXT("GeoScan_Benchmark_%s.csv"), *FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S"))));

		const TArray<FBenchmarkRow> Rows = Benchmark::Run(NumSeeds, StartSeed, CsvPath);
		int32 Planned = 0;
		int32 Flown = 0;
		for (const FBenchmarkRow& Row : Rows)
		{
			Planned += Row.bPlanSuccess ? 1 : 0;
			Flown += Row.bPlaybackSuccess ? 1 : 0;
		}
		UE_LOG(LogGeoScan, Log, TEXT("Benchmark: %d runs, planned %d, flown to boundary %d. Report: %s"),
			Rows.Num(), Planned, Flown, *CsvPath);
	}));

static FAutoConsoleCommandWithWorldAndArgs GeoScanDebugVizCmd(
	TEXT("GeoScan.DebugViz"),
	TEXT("Toggles debug visualization (SOI rings, planned route, maneuver markers). Args: 0|1"),
	FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
	{
		USimulationSubsystem* Sim = GetSim(World);
		if (Sim == nullptr)
		{
			return;
		}
		bool bEnable = !Sim->IsDebugVisualizationEnabled();
		if (Args.Num() > 0)
		{
			bEnable = FCString::Atoi(*Args[0]) != 0;
		}
		Sim->SetDebugVisualization(bEnable);
	}));

static FAutoConsoleCommandWithWorldAndArgs GeoScanSaveScenarioCmd(
	TEXT("GeoScan.SaveScenario"),
	TEXT("Saves the current scenario config to Saved/Scenarios/<Name>.json"),
	FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
	{
		USimulationSubsystem* Sim = GetSim(World);
		if (Sim == nullptr || Sim->GetScenario() == nullptr || Args.Num() == 0)
		{
			UE_LOG(LogGeoScan, Warning, TEXT("Usage: GeoScan.SaveScenario <name>"));
			return;
		}
		const FString Path = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Scenarios"), Args[0] + TEXT(".json"));
		UE_LOG(LogGeoScan, Log, TEXT("Scenario %s"),
			ScenarioJson::ExportToFile(Sim->GetScenario(), Path)
				? *FString::Printf(TEXT("saved to %s"), *Path)
				: TEXT("export FAILED"));
	}));

static FAutoConsoleCommandWithWorldAndArgs GeoScanLoadScenarioCmd(
	TEXT("GeoScan.LoadScenario"),
	TEXT("Loads Saved/Scenarios/<Name>.json and restarts the mission"),
	FConsoleCommandWithWorldAndArgsDelegate::CreateLambda([](const TArray<FString>& Args, UWorld* World)
	{
		USimulationSubsystem* Sim = GetSim(World);
		if (Sim == nullptr || Args.Num() == 0)
		{
			UE_LOG(LogGeoScan, Warning, TEXT("Usage: GeoScan.LoadScenario <name>"));
			return;
		}
		const FString Path = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Scenarios"), Args[0] + TEXT(".json"));
		UScenarioConfig* Config = ScenarioJson::ImportFromFile(Path, GetTransientPackage());
		if (Config == nullptr)
		{
			UE_LOG(LogGeoScan, Error, TEXT("Failed to import scenario from %s"), *Path);
			return;
		}
		Sim->StartScenario(Config);
	}));

static FAutoConsoleCommandWithWorld GeoScanReplanCmd(
	TEXT("GeoScan.Replan"),
	TEXT("Replans from the current spacecraft state"),
	FConsoleCommandWithWorldDelegate::CreateLambda([](UWorld* World)
	{
		if (USimulationSubsystem* Sim = GetSim(World))
		{
			Sim->ReplanMission();
		}
	}));
