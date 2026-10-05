#include "Sim/MissionRunner.h"
#include "Core/OrbitMath.h"

namespace MissionRunner
{

FMissionRunResult Run(const FPlanetarySystem& System, FSpacecraftState State,
	const FMissionPlan& Plan, const FSpacecraftFlightLimits& Limits,
	TArray<FVector>* OutWorldPath)
{
	FMissionRunResult Result;

	const double InitialFuel = State.FuelMass;

	TArray<FManeuverEvent> Pending = Plan.Maneuvers;
	Pending.Sort([](const FManeuverEvent& A, const FManeuverEvent& B) { return A.Time < B.Time; });

	const double Boundary = System.SystemBoundary;
	const double YearSeconds = 31557600.0;
	const double Horizon = Limits.MaxFlightTime > 0.0 ? Limits.MaxFlightTime : 100.0 * YearSeconds;

	auto Finish = [&](EMissionFailure Failure, double Time, const FVector& WorldPosition)
	{
		Result.Failure = Failure;
		Result.bSuccess = Failure == EMissionFailure::None;
		Result.FinishTime = Time;
		Result.FinalStarDistance = WorldPosition.Size();
		Result.FuelUsed = InitialFuel - State.FuelMass;
	};

	double Time = State.Epoch;
	constexpr int32 MaxIterations = 20000;
	for (int32 Iteration = 0; Iteration < MaxIterations; ++Iteration)
	{
		FVector WorldPosition;
		FVector WorldVelocity;
		if (!SpacecraftSim::GetWorldState(State, System, Time, WorldPosition, WorldVelocity))
		{
			Finish(EMissionFailure::None, Time, WorldPosition);
			return Result;
		}
		if (OutWorldPath)
		{
			OutWorldPath->Add(WorldPosition);
		}

		if (WorldPosition.Size() >= Boundary)
		{
			Finish(EMissionFailure::None, Time, WorldPosition);
			return Result;
		}

		const FPlanExecutionResult Execution = SpacecraftSim::ExecuteDueManeuvers(State, System, Pending, Time);
		Result.ManeuversExecuted += Execution.ExecutedCount;
		if (Execution.Failure != EMissionFailure::None)
		{
			Finish(Execution.Failure, Time, WorldPosition);
			return Result;
		}

		SpacecraftSim::UpdateDominantBody(State, System, Time);

		const EMissionFailure Constraint = SpacecraftSim::CheckFlightConstraints(State, System, Time, Limits);
		if (Constraint != EMissionFailure::None)
		{
			Finish(Constraint, Time, WorldPosition);
			return Result;
		}

		// Adaptive step: fine while inside a planet's SOI (flyby resolution), coarse outside.
		const FCelestialBodyDesc* Dominant = System.FindBody(State.DominantBodyId);
		double Step = 30.0 * 86400.0;
		if (Dominant && Dominant->ParentId != NAME_None)
		{
			Step = (2.0 * Dominant->SoiRadius / FMath::Max(State.Velocity.Size(), 1.0)) / 40.0;
		}
		else
		{
			Step = FMath::Min(Step, FMath::Max(3600.0, (Horizon - Time) / 1000.0));
		}

		Time += FMath::Max(Step, 60.0);
		if (Time > Horizon)
		{
			break;
		}
	}

	FVector WorldPosition;
	FVector WorldVelocity;
	SpacecraftSim::GetWorldState(State, System, Horizon, WorldPosition, WorldVelocity);
	Finish(EMissionFailure::TimeLimit, Horizon, WorldPosition);
	return Result;
}

} // namespace MissionRunner
