#include "Gen/SystemTypes.h"

void FCelestialBodyDesc::GetOrbitBasis(double InclinationRad, double LanRad, FVector& OutP, FVector& OutQ)
{
	const double CosO = FMath::Cos(LanRad);
	const double SinO = FMath::Sin(LanRad);
	const double CosI = FMath::Cos(InclinationRad);
	const double SinI = FMath::Sin(InclinationRad);

	OutP = FVector(CosO, SinO, 0.0);
	OutQ = FVector(-SinO * CosI, CosO * CosI, SinI);
}

void FCelestialBodyDesc::GetOrbitState(double Time, FVector& OutPosition, FVector& OutVelocity) const
{
	FVector P;
	FVector Q;
	GetOrbitBasis(Inclination, LAN, P, Q);

	const double Theta = InitialPhase + OrbitalAngularRate * Time;
	const double CosT = FMath::Cos(Theta);
	const double SinT = FMath::Sin(Theta);

	OutPosition = OrbitRadius * (CosT * P + SinT * Q);
	OutVelocity = OrbitRadius * OrbitalAngularRate * (CosT * Q - SinT * P);
}

const FCelestialBodyDesc* FPlanetarySystem::FindBody(FName BodyId) const
{
	for (const FCelestialBodyDesc& Body : Bodies)
	{
		if (Body.Id == BodyId)
		{
			return &Body;
		}
	}
	return nullptr;
}

const FCelestialBodyDesc& FPlanetarySystem::GetStar() const
{
	return Bodies[0];
}

bool FPlanetarySystem::GetBodyWorldState(FName BodyId, double Time, FVector& OutPosition, FVector& OutVelocity) const
{
	const FCelestialBodyDesc* Current = FindBody(BodyId);
	if (Current == nullptr)
	{
		return false;
	}

	OutPosition = FVector::ZeroVector;
	OutVelocity = FVector::ZeroVector;
	while (Current->ParentId != NAME_None)
	{
		const FCelestialBodyDesc* Parent = FindBody(Current->ParentId);
		if (Parent == nullptr)
		{
			return false;
		}

		FVector RelativePosition;
		FVector RelativeVelocity;
		Current->GetOrbitState(Time, RelativePosition, RelativeVelocity);
		OutPosition += RelativePosition;
		OutVelocity += RelativeVelocity;
		Current = Parent;
	}
	return true; // the star itself sits at the origin
}

double FPlanetarySystem::OutermostChildOrbitRadius() const
{
	double Outermost = 0.0;
	for (const FCelestialBodyDesc& Body : Bodies)
	{
		if (Body.ParentId != NAME_None)
		{
			Outermost = FMath::Max(Outermost, Body.OrbitRadius);
		}
	}
	return Outermost;
}
