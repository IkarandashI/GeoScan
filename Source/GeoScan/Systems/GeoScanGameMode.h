#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "GeoScanGameMode.generated.h"

class UScenarioConfig;

/**
 * Boots the default scenario into the simulation subsystem on play.
 * The asset reference is overridable per-BP/editor — mission parameters
 * themselves always come from the scenario DataAsset, never from code.
 */
UCLASS()
class GEOSCAN_API AGeoScanGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AGeoScanGameMode();

	virtual void BeginPlay() override;

	UPROPERTY(EditDefaultsOnly, Category = "GeoScan")
	TSoftObjectPtr<UScenarioConfig> DefaultScenario;
};
