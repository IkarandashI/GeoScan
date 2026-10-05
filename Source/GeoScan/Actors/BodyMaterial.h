#pragma once

#include "CoreMinimal.h"
#include "Materials/MaterialInterface.h"

/**
 * Shared parameterized body material (a small hand-made editor asset).
 * Parameters: "BodyColor" (vector), "EmissiveStrength" (scalar).
 * Returns nullptr when the asset does not exist — callers fall back to gray.
 */
inline UMaterialInterface* LoadBodyMaterial()
{
	return Cast<UMaterialInterface>(StaticLoadObject(UMaterialInterface::StaticClass(), nullptr,
		TEXT("/Game/Materials/M_BodyColor.M_BodyColor")));
}
