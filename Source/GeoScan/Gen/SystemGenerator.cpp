#include "Gen/SystemGenerator.h"
#include "Core/OrbitMath.h"
#include "Core/PCGRng.h"

namespace SystemGen
{

FPlanetarySystem Generate(const FSystemGenParams& Params)
{
	FPCG32 Rng(Params.Seed);
	FPlanetarySystem System;

	// Star: fixed center of the system.
	FCelestialBodyDesc Star;
	Star.Id = FName(TEXT("Star"));
	Star.TypeId = FName(TEXT("Star"));
	Star.ParentId = NAME_None;
	Star.Mass = Params.StarMass;
	Star.Mu = OrbitMath::GravitationalConstant * Star.Mass;
	Star.Radius = Params.StarRadius;
	System.Bodies.Add(Star);

	const int32 NumPlanets = FMath::Max(1, Params.NumPlanets);
	const double LogMin = FMath::Loge(Params.MinOrbitalRadius);
	const double LogMax = FMath::Loge(Params.MaxOrbitalRadius);
	const double MaxInclinationRad = FMath::DegreesToRadians(Params.MaxInclinationDeg);

	for (int32 Index = 0; Index < NumPlanets; ++Index)
	{
		// Geometric spacing keeps orbits monotonic and inside [Min, Max].
		const double T = NumPlanets > 1 ? static_cast<double>(Index) / (NumPlanets - 1) : 0.0;
		const double OrbitRadius = FMath::Exp(FMath::Lerp(LogMin, LogMax, T));

		FCelestialBodyDesc Planet;
		Planet.Id = FName(*FString::Printf(TEXT("Planet_%d"), Index + 1));
		Planet.TypeId = FName(TEXT("Planet"));
		Planet.ParentId = Star.Id;
		Planet.OrbitRadius = OrbitRadius;
		Planet.Mass = FMath::Exp(Rng.Range(FMath::Loge(Params.PlanetMassMin), FMath::Loge(Params.PlanetMassMax)));
		// Cube-root scaling keeps density roughly constant across the mass range.
		Planet.Radius = FMath::Clamp(
			Params.PlanetRadiusMin * FMath::Pow(Planet.Mass / Params.PlanetMassMin, 1.0 / 3.0),
			Params.PlanetRadiusMin, Params.PlanetRadiusMax);
		Planet.Mu = OrbitMath::GravitationalConstant * Planet.Mass;
		Planet.InitialPhase = Rng.Range(0.0, 2.0 * UE_DOUBLE_PI);
		Planet.LAN = 0.0;
		Planet.Inclination = Params.bInclinations ? Rng.Range(-MaxInclinationRad, MaxInclinationRad) : 0.0;
		System.Bodies.Add(Planet);
	}

	// Finalize computed fields that depend on the parent.
	for (FCelestialBodyDesc& Body : System.Bodies)
	{
		if (Body.ParentId == NAME_None)
		{
			continue;
		}
		const FCelestialBodyDesc* Parent = System.FindBody(Body.ParentId);
		Body.OrbitalAngularRate = FMath::Sqrt(Parent->Mu / (Body.OrbitRadius * Body.OrbitRadius * Body.OrbitRadius));
		Body.OrbitalPeriod = 2.0 * UE_DOUBLE_PI / Body.OrbitalAngularRate;
		Body.OrbitalSpeed = Body.OrbitalAngularRate * Body.OrbitRadius;
		Body.SoiRadius = OrbitMath::SphereOfInfluenceRadius(Body.Mu, Parent->Mu, Body.OrbitRadius);
	}

	System.SystemBoundary = Params.BoundaryMode == ESystemBoundaryMode::FixedDistance
		? Params.FixedBoundaryDistance
		: Params.BoundaryK * System.OutermostChildOrbitRadius();

	return System;
}

} // namespace SystemGen
