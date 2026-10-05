#pragma once

#include "CoreMinimal.h"
#include "Gen/SystemTypes.h"
#include "Sim/MissionPlan.h"
#include "Sim/SpacecraftState.h"

struct FMissionRunResult
{
	bool bSuccess = false;
	EMissionFailure Failure = EMissionFailure::None;
	double FinishTime = 0.0;      // s, boundary crossing time on success
	double FinalStarDistance = 0.0;
	double FuelUsed = 0.0;
	int32 ManeuversExecuted = 0;
};

namespace MissionRunner
{
	/**
	 * Pure patched-conics playback of a plan: adaptive steps (fine inside SOIs,
	 * coarse heliocentric), maneuvers at their exact event times, SOI switching,
	 * hard constraints, boundary success. Used by automation tests and the
	 * benchmark to verify plans end-to-end without a running world.
	 * When OutWorldPath is set, star-centered positions (meters) are recorded per step.
	 */
	FMissionRunResult Run(const FPlanetarySystem& System, FSpacecraftState State,
		const FMissionPlan& Plan, const FSpacecraftFlightLimits& Limits,
		TArray<FVector>* OutWorldPath = nullptr);
}
