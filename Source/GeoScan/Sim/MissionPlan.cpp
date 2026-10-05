#include "Sim/MissionPlan.h"
#include "Core/OrbitMath.h"

namespace SpacecraftSim
{

FPlanExecutionResult ExecuteDueManeuvers(FSpacecraftState& State, const FPlanetarySystem& System,
	TArray<FManeuverEvent>& PendingManeuvers, double SimTime)
{
	FPlanExecutionResult Result;
	while (!PendingManeuvers.IsEmpty() && PendingManeuvers[0].Time <= SimTime)
	{
		const FManeuverEvent Event = PendingManeuvers[0];
		if (!ApplyImpulse(State, System, Event.Time, Event.DeltaV))
		{
			// Hard rule from the task: cannot burn more fuel than available.
			Result.Failure = EMissionFailure::Fuel;
			return Result;
		}
		PendingManeuvers.RemoveAt(0);
		++Result.ExecutedCount;
	}
	return Result;
}

EMissionFailure CheckFlightConstraints(const FSpacecraftState& State, const FPlanetarySystem& System,
	double Time, const FSpacecraftFlightLimits& Limits)
{
	FVector WorldPosition;
	FVector WorldVelocity;
	if (!GetWorldState(State, System, Time, WorldPosition, WorldVelocity))
	{
		return EMissionFailure::None;
	}

	if (Limits.MaxSpeed > 0.0 && WorldVelocity.Size() > Limits.MaxSpeed)
	{
		return EMissionFailure::SpeedLimit;
	}
	if (Limits.MaxFlightTime > 0.0 && Time > Limits.MaxFlightTime)
	{
		return EMissionFailure::TimeLimit;
	}

	for (const FCelestialBodyDesc& Body : System.Bodies)
	{
		FVector BodyPosition;
		FVector BodyVelocity;
		if (!System.GetBodyWorldState(Body.Id, Time, BodyPosition, BodyVelocity))
		{
			continue;
		}
		if ((WorldPosition - BodyPosition).Size() < Body.Radius)
		{
			return EMissionFailure::Collision;
		}
	}
	return EMissionFailure::None;
}

} // namespace SpacecraftSim
