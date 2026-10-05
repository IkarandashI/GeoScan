#include "Actors/CelestialBodyActor.h"

#include "Actors/BodyMaterial.h"
#include "Components/LineBatchComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Materials/MaterialInstanceDynamic.h"
#include "Systems/SimulationSubsystem.h"
#include "UObject/ConstructorHelpers.h"

namespace
{
	// Deterministic per-body color: hue from a golden-angle sequence keyed by the
	// body id — stable across runs, distinct between bodies.
	FLinearColor ComputeBodyColor(const FCelestialBodyDesc& Body)
	{
		if (Body.ParentId == NAME_None)
		{
			return FLinearColor(1.0f, 0.82f, 0.45f); // warm star
		}
		const uint32 Hash = GetTypeHash(Body.Id.ToString());
		const float Hue = FMath::Fmod(Hash * 0.618034f, 1.0f);
		const float Saturation = 0.4f + 0.25f * FMath::Fmod(Hash * 0.381966f, 1.0f);
		return FLinearColor(Hue, Saturation, 0.55f).HSVToLinearRGB();
	}
}

ACelestialBodyActor::ACelestialBodyActor()
{
	PrimaryActorTick.bCanEverTick = true;

	Mesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("Mesh"));
	SetRootComponent(Mesh);
	Mesh->SetCollisionEnabled(ECollisionEnabled::NoCollision);

	LineBatcher = CreateDefaultSubobject<ULineBatchComponent>(TEXT("LineBatcher"));
}

void ACelestialBodyActor::Init(USimulationSubsystem* InSim, const FCelestialBodyDesc& InBody)
{
	Sim = InSim;
	Body = InBody;

	// The mesh (and material) come from the Blueprint/asset defaults; C++ only
	// sizes whatever is set. No mesh set means no visual — the orbit line still
	// marks the body.
	FVector BoundsMin;
	FVector BoundsMax;
	Mesh->GetLocalBounds(BoundsMin, BoundsMax);
	const double MeshRadiusUU = FMath::Max(1.0, static_cast<double>(((BoundsMax - BoundsMin) * 0.5).GetMax()));
	const double VisualRadiusUU = Sim->GetVisualScale().BodyVisualRadiusUU(Body);
	Mesh->SetWorldScale3D(FVector(VisualRadiusUU / MeshRadiusUU));

	if (UMaterialInterface* BodyMaterial = LoadBodyMaterial())
	{
		// Null when the actor has no mesh/material asset defaults (allowed since
		// visuals live in assets) — tinting is simply skipped then.
		if (UMaterialInstanceDynamic* DynamicMaterial = Mesh->CreateAndSetMaterialInstanceDynamic(0))
		{
			DynamicMaterial->SetVectorParameterValue(TEXT("BodyColor"), ComputeBodyColor(Body));
			// The star self-illuminates; planets keep a faint rim so the night side
			// does not go pitch black at system-view distances.
			DynamicMaterial->SetScalarParameterValue(TEXT("EmissiveStrength"),
				Body.ParentId == NAME_None ? 3.0f : 0.08f);
		}
	}

	UE_LOG(LogTemp, Log, TEXT("[Body] %s: class=%s mesh=%s base material=%s"),
		*Body.Id.ToString(), *GetClass()->GetName(),
		Mesh->GetStaticMesh() ? *Mesh->GetStaticMesh()->GetName() : TEXT("NONE"),
		Mesh->GetMaterial(0) ? *Mesh->GetMaterial(0)->GetName() : TEXT("NONE"));

	DrawOrbit();
}

void ACelestialBodyActor::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!Sim.IsValid())
	{
		return;
	}

	FVector Position;
	FVector Velocity;
	if (Sim->GetSystem().GetBodyWorldState(Body.Id, Sim->GetSimTime(), Position, Velocity))
	{
		SetActorLocation(Sim->GetVisualScale().ToWorldUnits(Position));
	}
}

void ACelestialBodyActor::DrawOrbit()
{
	if (Body.ParentId == NAME_None || !Sim.IsValid())
	{
		return; // the star sits at the origin; it has no orbit to draw
	}

	const FPlanetarySystem& System = Sim->GetSystem();
	const FCelestialBodyDesc* Parent = System.FindBody(Body.ParentId);
	if (Parent == nullptr)
	{
		return;
	}

	// Circle in the body's orbital plane around the (static) star position.
	FVector P;
	FVector Q;
	FCelestialBodyDesc::GetOrbitBasis(Body.Inclination, Body.LAN, P, Q);

	constexpr int32 Segments = 96;
	const double RadiusUU = Sim->GetVisualScale().ScaleFactor() * Body.OrbitRadius;
	const FLinearColor Color(0.25f, 0.27f, 0.32f, 1.0f);

	FVector Previous;
	for (int32 Index = 0; Index <= Segments; ++Index)
	{
		const double Theta = 2.0 * UE_DOUBLE_PI * Index / Segments;
		const FVector Point = RadiusUU * (FMath::Cos(Theta) * P + FMath::Sin(Theta) * Q);
		if (Index > 0)
		{
			LineBatcher->DrawLine(Previous, Point, Color, 1.0f, 0.0f);
		}
		Previous = Point;
	}
}
