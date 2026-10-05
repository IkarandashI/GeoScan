#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Actor.h"
#include "SpacecraftActor.generated.h"

class ULineBatchComponent;
class USimulationSubsystem;
class UStaticMeshComponent;

/** Visual representation of the spacecraft plus its predicted ballistic trajectory. */
UCLASS()
class GEOSCAN_API ASpacecraftActor : public AActor
{
	GENERATED_BODY()

public:
	ASpacecraftActor();

	void Init(USimulationSubsystem* InSim);

	virtual void Tick(float DeltaTime) override;

private:
	void DrawTrajectory();

	UPROPERTY(VisibleAnywhere, Category = "GeoScan")
	TObjectPtr<UStaticMeshComponent> Mesh;

	UPROPERTY(VisibleAnywhere, Category = "GeoScan")
	TObjectPtr<ULineBatchComponent> LineBatcher;

	TWeakObjectPtr<USimulationSubsystem> Sim;

	// Redraw only when the conic snapshot changes (SOI switch or maneuver).
	FName LastDrawnDominantBody;
	double LastDrawnEpoch = -1.0;
};
