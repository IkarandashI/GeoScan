#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "OrbitCameraPawn.generated.h"

class UCameraComponent;
class USimulationSubsystem;

/**
 * Free orbital camera: rotates/pans/zooms around a focus target (spacecraft or
 * any celestial body). Also owns the time-warp hotkeys (0-8).
 */
UCLASS()
class GEOSCAN_API AOrbitCameraPawn : public APawn
{
	GENERATED_BODY()

public:
	AOrbitCameraPawn();

	virtual void Tick(float DeltaTime) override;
	virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

protected:
	void OnLookYaw(float AxisValue);
	void OnLookPitch(float AxisValue);
	void OnZoom(float AxisValue);
	void OnMoveForward(float AxisValue);
	void OnMoveRight(float AxisValue);
	void OnMoveUp(float AxisValue);
	void OnFocusNext();
	void OnFocusSpacecraft();
	void OnResetView();
	void OnReplan();
	void OnToggleDebugViz();
	void OnWarpKey(FKey Key);
	void SetWarp(int32 WarpIndex);

private:
	FVector ComputeFocusPosition() const;
	double DefaultDistanceForFocus() const;
	bool bRotateEnabled() const;

	UPROPERTY(VisibleAnywhere, Category = "GeoScan")
	TObjectPtr<UCameraComponent> Camera;

	TWeakObjectPtr<USimulationSubsystem> Sim;

	// Camera state.
	double YawDeg = -45.0;
	double PitchDeg = -30.0;
	double DistanceUU = 4000.0;
	FVector FocusOffsetUU = FVector::ZeroVector;
	FVector2D MoveInput = FVector2D::ZeroVector;
	double VerticalInput = 0.0;

	/** Focus 0 = spacecraft, then bodies in system order. Starts on the star (system overview). */
	int32 FocusIndex = 1;
	bool bFocusDistanceInitialized = false;
};
