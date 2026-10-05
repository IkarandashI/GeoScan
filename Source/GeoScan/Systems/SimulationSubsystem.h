#pragma once

#include "CoreMinimal.h"
#include "Gen/SystemTypes.h"
#include "Sim/MissionPlan.h"
#include "Sim/SpacecraftState.h"
#include "Sim/VisualScale.h"
#include "Subsystems/WorldSubsystem.h"
#include "SimulationSubsystem.generated.h"

class ACelestialBodyActor;
class ASpacecraftActor;
class ULineBatchComponent;
class UScenarioConfig;

UENUM(BlueprintType)
enum class EMissionStatus : uint8
{
	Idle,
	Running,
	Success,
	FailedCollision,
	FailedFuel,
	FailedTime,
	FailedSpeed
};

/**
 * Owns the running mission: generated system, simulation clock with time warp,
 * spacecraft state and spawned world actors. Actors and HUD read from here.
 */
UCLASS()
class GEOSCAN_API USimulationSubsystem : public UTickableWorldSubsystem
{
	GENERATED_BODY()

public:
	virtual void Initialize(FSubsystemCollectionBase& Collection) override;
	virtual void Deinitialize() override;
	virtual void Tick(float DeltaTime) override;
	virtual TStatId GetStatId() const override;

	/** Generates the system from the scenario, validates it and spawns world actors. */
	bool StartScenario(UScenarioConfig* Config);

	bool IsRunning() const { return Status == EMissionStatus::Running; }
	EMissionStatus GetStatus() const { return Status; }
	double GetSimTime() const { return SimTime; }
	double GetSystemBoundary() const { return System.SystemBoundary; }
	const FPlanetarySystem& GetSystem() const { return System; }
	const FSpacecraftState& GetSpacecraft() const { return Spacecraft; }
	UScenarioConfig* GetScenario() const { return Scenario; }
	const FVisualScaleSettings& GetVisualScale() const { return VisualScale; }
	const FString& GetFailureReason() const { return FailureReason; }

	bool GetSpacecraftWorldState(FVector& OutPosition, FVector& OutVelocity) const;

	/** Visual (world-units) spacecraft position, remapped near bodies so close
	 *  orbits render outside the oversized body balls. */
	bool GetSpacecraftVisualPosition(FVector& OutPositionUU) const;

	/** Loads a maneuver plan for the autopilot (replaces any previous plan). */
	void LoadPlan(const FMissionPlan& Plan);
	void ClearPlan();
	int32 GetExecutedManeuverCount() const { return ExecutedManeuvers; }
	int32 GetPendingManeuverCount() const { return PendingManeuvers.Num(); }
	bool GetNextManeuver(FManeuverEvent& OutEvent) const;
	const FMissionPlan& GetLoadedPlan() const { return LoadedPlan; }
	const TArray<FName>& GetRouteBodies() const { return RouteBodies; }

	/** Plans from the initial state and loads the route (called automatically on start). */
	bool PlanMission();

	/** Replans from the CURRENT spacecraft state — mid-flight route change (task bonus). */
	bool ReplanMission();

	/** Debug visualization: SOI rings, planned route polyline, maneuver markers. */
	void SetDebugVisualization(bool bEnabled);
	bool IsDebugVisualizationEnabled() const { return bDebugVisualization; }

	/** Time warp: index into GetWarpLevels() (0 = paused). */
	static const TArray<double>& GetWarpLevels();
	int32 GetWarpIndex() const;
	void SetWarpIndex(int32 Index);

private:
	void SpawnActors();
	void FinishMission(EMissionStatus InStatus, const FString& Reason);
	void UpdateDebugVisualization();

	EMissionStatus Status = EMissionStatus::Idle;
	FString FailureReason;
	double SimTime = 0.0;
	double TimeWarp = 0.0; // starts paused; player ramps up explicitly

	UPROPERTY()
	TObjectPtr<UScenarioConfig> Scenario;

	FPlanetarySystem System;
	FSpacecraftState Spacecraft;
	FSpacecraftFlightLimits FlightLimits;
	FMissionPlan LoadedPlan;
	TArray<FManeuverEvent> PendingManeuvers;
	TArray<FName> RouteBodies;
	int32 ExecutedManeuvers = 0;
	FVisualScaleSettings VisualScale;

	TArray<TWeakObjectPtr<ACelestialBodyActor>> BodyActors;
	TWeakObjectPtr<ASpacecraftActor> SpacecraftActor;

	bool bDebugVisualization = false;
	double LastVizEpoch = -1.0;
	int32 LastVizPendingCount = -1;
	TWeakObjectPtr<AActor> DebugBatchHost;
	TWeakObjectPtr<ULineBatchComponent> DebugBatcher;

	ULineBatchComponent* GetDebugBatcher();
};
