#pragma once

#include "CoreMinimal.h"
#include "Gen/SystemTypes.h"
#include "GameFramework/Actor.h"
#include "CelestialBodyActor.generated.h"

class ULineBatchComponent;
class USimulationSubsystem;
class UStaticMeshComponent;

/**
 * Visual representation of one celestial body. Position each tick comes from
 * the analytic ephemeris — never simulated, always exact at the current SimTime.
 */
UCLASS()
class GEOSCAN_API ACelestialBodyActor : public AActor
{
	GENERATED_BODY()

public:
	ACelestialBodyActor();

	void Init(USimulationSubsystem* InSim, const FCelestialBodyDesc& InBody);

	virtual void Tick(float DeltaTime) override;

	const FCelestialBodyDesc& GetBody() const { return Body; }

	/** Mesh component accessor (diagnostics/tests). */
	UStaticMeshComponent* GetBodyMesh() const { return Mesh; }

protected:
	UPROPERTY(VisibleAnywhere, Category = "GeoScan")
	TObjectPtr<UStaticMeshComponent> Mesh;

	UPROPERTY(VisibleAnywhere, Category = "GeoScan")
	TObjectPtr<ULineBatchComponent> LineBatcher;

private:
	void DrawOrbit();

	FCelestialBodyDesc Body;
	TWeakObjectPtr<USimulationSubsystem> Sim;
};
