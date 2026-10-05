#pragma once

#include "CoreMinimal.h"
#include "GameFramework/HUD.h"
#include "SimHUD.generated.h"

/** Minimal canvas HUD: mission clock, telemetry, controls. No UMG assets needed. */
UCLASS()
class GEOSCAN_API ASimHUD : public AHUD
{
	GENERATED_BODY()

public:
	virtual void DrawHUD() override;
};
