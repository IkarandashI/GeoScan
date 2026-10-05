#include "Systems/GeoScanGameMode.h"

#include "Actors/OrbitCameraPawn.h"
#include "Config/ScenarioConfig.h"
#include "Systems/SimulationSubsystem.h"
#include "UI/SimHUD.h"

AGeoScanGameMode::AGeoScanGameMode()
{
	DefaultPawnClass = AOrbitCameraPawn::StaticClass();
	HUDClass = ASimHUD::StaticClass();

	DefaultScenario = FSoftObjectPath(TEXT("/Game/Scenarios/DA_Scenario_Balanced.DA_Scenario_Balanced"));
}

void AGeoScanGameMode::BeginPlay()
{
	Super::BeginPlay();

	UScenarioConfig* Scenario = DefaultScenario.LoadSynchronous();
	if (Scenario == nullptr)
	{
		UE_LOG(LogTemp, Error, TEXT("GeoScanGameMode: default scenario asset not found"));
		return;
	}

	if (USimulationSubsystem* Sim = GetWorld()->GetSubsystem<USimulationSubsystem>())
	{
		Sim->StartScenario(Scenario);
	}
}
