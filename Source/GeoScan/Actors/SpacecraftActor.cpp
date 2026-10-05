#include "Actors/SpacecraftActor.h"

#include "Actors/BodyMaterial.h"
#include "Components/LineBatchComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Core/OrbitMath.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Sim/SpacecraftState.h"
#include "Systems/SimulationSubsystem.h"

namespace
{
	constexpr int32 TrajectorySamples = 96;
}

ASpacecraftActor::ASpacecraftActor()
{
	PrimaryActorTick.bCanEverTick = true;

	Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Mesh"));
	SetRootComponent(Mesh);
	Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

	LineBatcher = CreateDefaultSubobject<ULineBatchComponent>(TEXT("LineBatcher"));
}

void ASpacecraftActor::Init(USimulationSubsystem* InSim)
{
	Sim = InSim;

	// The mesh (and material) come from the Blueprint/asset defaults; C++ only
	// sizes whatever is set. No mesh set means no visual — the trajectory line
	// still marks the spacecraft.
	FVector BoundsMin;
	FVector BoundsMax;
	Mesh->GetLocalBounds(BoundsMin, BoundsMax);
	const double MeshRadiusUU = FMath::Max(1.0, static_cast<double>(((BoundsMax - BoundsMin) * 0.5).GetMax()));
	const double VisualRadiusUU = Sim->GetVisualScale().SpacecraftVisualRadiusUU;
	Mesh->SetWorldScale3D(FVector(VisualRadiusUU / MeshRadiusUU));

	if (UMaterialInterface* BodyMaterial = LoadBodyMaterial())
	{
		if (UMaterialInstanceDynamic* DynamicMaterial = Mesh->CreateAndSetMaterialInstanceDynamic(0))
		{
			DynamicMaterial->SetVectorParameterValue(TEXT("BodyColor"), FLinearColor(0.1f, 0.8f, 1.0f));
			DynamicMaterial->SetScalarParameterValue(TEXT("EmissiveStrength"), 1.5f);
		}
	}

	DrawTrajectory();
}

void ASpacecraftActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!Sim.IsValid())
	{
		return;
	}

	const FSpacecraftState& Spacecraft = Sim->GetSpacecraft();
	if (Spacecraft.DominantBodyId != LastDrawnDominantBody || Spacecraft.Epoch != LastDrawnEpoch)
	{
		DrawTrajectory();
	}

	FVector VisualPosition;
	if (Sim->GetSpacecraftVisualPosition(VisualPosition))
	{
		SetActorLocation(VisualPosition);
	}
}

void ASpacecraftActor::DrawTrajectory()
{
	if (!Sim.IsValid())
	{
		return;
	}
	LineBatcher->Flush();

	const FPlanetarySystem& System = Sim->GetSystem();
	const FSpacecraftState& Spacecraft = Sim->GetSpacecraft();
	const FCelestialBodyDesc* Body = System.FindBody(Spacecraft.DominantBodyId);
	if (Body == nullptr)
	{
		return;
	}

	// Sample one revolution for bound orbits, a fixed horizon for escape arcs.
	const double SemiMajorAxis = OrbitMath::SemiMajorAxis(Spacecraft.Position, Spacecraft.Velocity, Body->Mu);
	const double Horizon = SemiMajorAxis > 0.0
		? 1.02 * OrbitMath::OrbitalPeriod(Body->Mu, SemiMajorAxis)
		: 86400.0 * 200.0;

	const FVisualScaleSettings& Scale = Sim->GetVisualScale();
	const FLinearColor Color(0.15f, 0.75f, 0.95f, 1.0f);

	FVector PreviousPoint;
	bool bHasPrevious = false;
	for (int32 Index = 0; Index <= TrajectorySamples; ++Index)
	{
		const double Time = Spacecraft.Epoch + Horizon * Index / TrajectorySamples;

		FVector RelativePosition;
		FVector IgnoredVelocity;
		OrbitMath::Propagate(Spacecraft.Position, Spacecraft.Velocity, Body->Mu, Time - Spacecraft.Epoch,
			RelativePosition, IgnoredVelocity);

		FVector BodyPosition;
		FVector BodyVelocity;
		if (!System.GetBodyWorldState(Body->Id, Time, BodyPosition, BodyVelocity))
		{
			return;
		}

		// Remap close-range distances so the arc renders outside the oversized ball.
		const FVector PointUU = Scale.ToWorldUnits(BodyPosition)
			+ Scale.ToWorldUnitsAroundBody(*Body, RelativePosition);
		if (bHasPrevious)
		{
			LineBatcher->DrawLine(PreviousPoint, PointUU, Color, 2.0f, 0.0f);
		}
		PreviousPoint = PointUU;
		bHasPrevious = true;
	}

	LastDrawnDominantBody = Spacecraft.DominantBodyId;
	LastDrawnEpoch = Spacecraft.Epoch;
}
