#pragma once

#include "CoreMinimal.h"

class UScenarioConfig;

/**
 * Stable hand-rolled JSON format for scenario configs (test task bonus:
 * save/load scenario). Field-by-field mapping keeps the format diff-friendly
 * in git and independent from UObject serialization internals.
 */
namespace ScenarioJson
{
	/** Serializes the config and writes it to FilePath. Returns false on IO errors. */
	GEOSCAN_API bool ExportToFile(const UScenarioConfig* Config, const FString& FilePath);

	/** Creates a new config object filled from FilePath. Returns nullptr on parse errors. */
	GEOSCAN_API UScenarioConfig* ImportFromFile(const FString& FilePath, UObject* Outer);
}
