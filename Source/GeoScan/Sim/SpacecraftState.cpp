#include "Sim/SpacecraftState.h"
#include "Core/OrbitMath.h"

double FSpacecraftState::DeltaVRemaining() const
{
	return OrbitMath::DeltaVBudget(TotalMass(), DryMass, Isp);
}

namespace SpacecraftSim
{

FSpacecraftState MakeInitialState(const FPlanetarySystem& System, const FSpacecraftFlightParams& Params)
{
	const FCelestialBodyDesc* Body = System.FindBody(Params.StartingBodyId);
	// The scenario was validated beforehand; fall back to the star defensively.
	const FCelestialBodyDesc& Resolved = Body ? *Body : System.GetStar();

	FVector P;
	FVector Q;
	FCelestialBodyDesc::GetOrbitBasis(Resolved.Inclination, Resolved.LAN, P, Q);

	const double CosPhi = FMath::Cos(Params.InitialOrbitPhase);
	const double SinPhi = FMath::Sin(Params.InitialOrbitPhase);
	const double Speed = OrbitMath::CircularOrbitSpeed(Resolved.Mu, Params.StartingOrbitRadius);

	FSpacecraftState State;
	State.DominantBodyId = Resolved.Id;
	State.Epoch = 0.0;
	State.Position = Params.StartingOrbitRadius * (CosPhi * P + SinPhi * Q);
	State.Velocity = Speed * (CosPhi * Q - SinPhi * P);
	State.DryMass = Params.DryMass;
	State.FuelMass = Params.FuelMass;
	State.MaxThrust = Params.MaxThrust;
	State.Isp = Params.Isp;
	return State;
}

bool GetWorldState(const FSpacecraftState& State, const FPlanetarySystem& System, double Time,
	FVector& OutPosition, FVector& OutVelocity)
{
	const FCelestialBodyDesc* Body = System.FindBody(State.DominantBodyId);
	if (Body == nullptr)
	{
		return false;
	}

	FVector RelativePosition;
	FVector RelativeVelocity;
	OrbitMath::Propagate(State.Position, State.Velocity, Body->Mu, Time - State.Epoch, RelativePosition, RelativeVelocity);

	FVector BodyPosition;
	FVector BodyVelocity;
	if (!System.GetBodyWorldState(Body->Id, Time, BodyPosition, BodyVelocity))
	{
		return false;
	}

	OutPosition = BodyPosition + RelativePosition;
	OutVelocity = BodyVelocity + RelativeVelocity;
	return true;
}

void UpdateDominantBody(FSpacecraftState& State, const FPlanetarySystem& System, double Time)
{
	for (int32 Step = 0; Step < 8; ++Step)
	{
		const FCelestialBodyDesc* Body = System.FindBody(State.DominantBodyId);
		if (Body == nullptr)
		{
			return;
		}

		FVector RelativePosition;
		FVector RelativeVelocity;
		OrbitMath::Propagate(State.Position, State.Velocity, Body->Mu, Time - State.Epoch, RelativePosition, RelativeVelocity);

		FVector BodyPosition;
		FVector BodyVelocity;
		if (!System.GetBodyWorldState(Body->Id, Time, BodyPosition, BodyVelocity))
		{
			return;
		}
		const FVector WorldPosition = BodyPosition + RelativePosition;
		const FVector WorldVelocity = BodyVelocity + RelativeVelocity;

		// Promote: left the current body's sphere of influence.
		if (Body->ParentId != NAME_None && Body->SoiRadius > 0.0 && RelativePosition.Size() > Body->SoiRadius)
		{
			FVector ParentPosition;
			FVector ParentVelocity;
			if (!System.GetBodyWorldState(Body->ParentId, Time, ParentPosition, ParentVelocity))
			{
				return;
			}
			State.DominantBodyId = Body->ParentId;
			State.Position = WorldPosition - ParentPosition;
			State.Velocity = WorldVelocity - ParentVelocity;
			State.Epoch = Time;
			continue;
		}

		// Descend: entered the sphere of influence of one of the children.
		bool bDescended = false;
		for (const FCelestialBodyDesc& Child : System.Bodies)
		{
			if (Child.ParentId != Body->Id || Child.SoiRadius <= 0.0)
			{
				continue;
			}
			FVector ChildPosition;
			FVector ChildVelocity;
			if (!System.GetBodyWorldState(Child.Id, Time, ChildPosition, ChildVelocity))
			{
				continue;
			}
			if ((WorldPosition - ChildPosition).Size() < Child.SoiRadius)
			{
				State.DominantBodyId = Child.Id;
				State.Position = WorldPosition - ChildPosition;
				State.Velocity = WorldVelocity - ChildVelocity;
				State.Epoch = Time;
				bDescended = true;
				break;
			}
		}
		if (!bDescended)
		{
			return;
		}
	}
}

bool ApplyImpulse(FSpacecraftState& State, const FPlanetarySystem& System, double Time, const FVector& DeltaV)
{
	const FCelestialBodyDesc* Body = System.FindBody(State.DominantBodyId);
	if (Body == nullptr)
	{
		return false;
	}

	const double FuelNeeded = OrbitMath::FuelBurned(State.TotalMass(), DeltaV.Size(), State.Isp);
	if (FuelNeeded > State.FuelMass)
	{
		return false; // not enough fuel — the state stays exactly as it was
	}

	OrbitMath::Propagate(State.Position, State.Velocity, Body->Mu, Time - State.Epoch, State.Position, State.Velocity);
	State.Epoch = Time;
	State.FuelMass -= FuelNeeded;
	State.Velocity += DeltaV;
	return true;
}

} // namespace SpacecraftSim
