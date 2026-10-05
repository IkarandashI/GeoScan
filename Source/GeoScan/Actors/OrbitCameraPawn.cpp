#include "Actors/OrbitCameraPawn.h"

#include "Camera/CameraComponent.h"
#include "Components/InputComponent.h"
#include "GameFramework/PlayerController.h"
#include "Systems/SimulationSubsystem.h"

AOrbitCameraPawn::AOrbitCameraPawn()
{
	PrimaryActorTick.bCanEverTick = true;

	USceneComponent* Root = CreateDefaultSubobject<USceneComponent>(TEXT("Root"));
	SetRootComponent(Root);

	Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
	Camera->SetupAttachment(Root);
	Camera->SetFieldOfView(70.0f);
}

void AOrbitCameraPawn::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

	if (!Sim.IsValid())
	{
		Sim = GetWorld() ? GetWorld()->GetSubsystem<USimulationSubsystem>() : nullptr;
	}
	if (!Sim.IsValid())
	{
		return;
	}

	// One-time default framing: pull back far enough to see the outermost orbit.
	if (!bFocusDistanceInitialized && Sim->GetSystem().Bodies.Num() > 0)
	{
		bFocusDistanceInitialized = true;
		const FVisualScaleSettings& Scale = Sim->GetVisualScale();
		double OutermostUU = 1000.0;
		for (const FCelestialBodyDesc& Body : Sim->GetSystem().Bodies)
		{
			OutermostUU = FMath::Max(OutermostUU, Body.OrbitRadius * Scale.ScaleFactor());
		}
		DistanceUU = FMath::Clamp(OutermostUU * 1.15, 3000.0, 200000.0);
	}

	// Pan in the camera's horizontal frame; speed scales with distance to the focus.
	if (!MoveInput.IsNearlyZero() || !FMath::IsNearlyZero(VerticalInput))
	{
		const FVector FlatForward = (Camera->GetForwardVector() * FVector(1.0, 1.0, 0.0)).GetSafeNormal();
		const FVector Right = FVector::CrossProduct(FVector::UpVector, FlatForward);
		const double PanSpeed = DistanceUU * 1.2 * DeltaTime;
		FocusOffsetUU += (FlatForward * MoveInput.Y + Right * MoveInput.X + FVector::UpVector * VerticalInput) * PanSpeed;
		MoveInput = FVector2D::ZeroVector;
		VerticalInput = 0.0;
	}

	const FVector Focus = ComputeFocusPosition() + FocusOffsetUU;

	const double YawRad = FMath::DegreesToRadians(YawDeg);
	const double PitchRad = FMath::DegreesToRadians(PitchDeg);
	const FVector Offset(
		FMath::Cos(PitchRad) * FMath::Cos(YawRad),
		FMath::Cos(PitchRad) * FMath::Sin(YawRad),
		FMath::Sin(PitchRad));

	SetActorLocation(Focus + Offset * DistanceUU);
	Camera->SetWorldRotation((Focus - GetActorLocation()).Rotation());
}

void AOrbitCameraPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
	Super::SetupPlayerInputComponent(PlayerInputComponent);

	PlayerInputComponent->BindAxis(TEXT("LookYaw"), this, &AOrbitCameraPawn::OnLookYaw);
	PlayerInputComponent->BindAxis(TEXT("LookPitch"), this, &AOrbitCameraPawn::OnLookPitch);
	PlayerInputComponent->BindAxis(TEXT("Zoom"), this, &AOrbitCameraPawn::OnZoom);
	PlayerInputComponent->BindAxis(TEXT("CamForward"), this, &AOrbitCameraPawn::OnMoveForward);
	PlayerInputComponent->BindAxis(TEXT("CamRight"), this, &AOrbitCameraPawn::OnMoveRight);
	PlayerInputComponent->BindAxis(TEXT("CamUpDown"), this, &AOrbitCameraPawn::OnMoveUp);

	PlayerInputComponent->BindAction(TEXT("FocusNext"), IE_Pressed, this, &AOrbitCameraPawn::OnFocusNext);
	PlayerInputComponent->BindKey(EKeys::C, IE_Pressed, this, &AOrbitCameraPawn::OnFocusSpacecraft);
	PlayerInputComponent->BindAction(TEXT("ResetView"), IE_Pressed, this, &AOrbitCameraPawn::OnResetView);
	PlayerInputComponent->BindAction(TEXT("Replan"), IE_Pressed, this, &AOrbitCameraPawn::OnReplan);
	PlayerInputComponent->BindKey(EKeys::V, IE_Pressed, this, &AOrbitCameraPawn::OnToggleDebugViz);

	PlayerInputComponent->BindKey(EKeys::Zero, IE_Pressed, this, &AOrbitCameraPawn::OnWarpKey);
	PlayerInputComponent->BindKey(EKeys::One, IE_Pressed, this, &AOrbitCameraPawn::OnWarpKey);
	PlayerInputComponent->BindKey(EKeys::Two, IE_Pressed, this, &AOrbitCameraPawn::OnWarpKey);
	PlayerInputComponent->BindKey(EKeys::Three, IE_Pressed, this, &AOrbitCameraPawn::OnWarpKey);
	PlayerInputComponent->BindKey(EKeys::Four, IE_Pressed, this, &AOrbitCameraPawn::OnWarpKey);
	PlayerInputComponent->BindKey(EKeys::Five, IE_Pressed, this, &AOrbitCameraPawn::OnWarpKey);
	PlayerInputComponent->BindKey(EKeys::Six, IE_Pressed, this, &AOrbitCameraPawn::OnWarpKey);
	PlayerInputComponent->BindKey(EKeys::Seven, IE_Pressed, this, &AOrbitCameraPawn::OnWarpKey);
	PlayerInputComponent->BindKey(EKeys::Eight, IE_Pressed, this, &AOrbitCameraPawn::OnWarpKey);
}

void AOrbitCameraPawn::OnWarpKey(FKey Key)
{
	struct FKeyToIndex
	{
		FKey Key;
		int32 Index;
	};
	static const FKeyToIndex Mappings[] = {
		{ EKeys::Zero, 0 }, { EKeys::One, 1 }, { EKeys::Two, 2 }, { EKeys::Three, 3 },
		{ EKeys::Four, 4 }, { EKeys::Five, 5 }, { EKeys::Six, 6 }, { EKeys::Seven, 7 },
		{ EKeys::Eight, 8 },
	};
	for (const FKeyToIndex& Mapping : Mappings)
	{
		if (Mapping.Key == Key)
		{
			SetWarp(Mapping.Index);
			return;
		}
	}
}

bool AOrbitCameraPawn::bRotateEnabled() const
{
	const APlayerController* PlayerController = Cast<APlayerController>(GetController());
	return PlayerController && PlayerController->IsInputKeyDown(EKeys::RightMouseButton);
}

void AOrbitCameraPawn::OnLookYaw(float AxisValue)
{
	if (bRotateEnabled())
	{
		YawDeg += AxisValue * 0.35;
	}
}

void AOrbitCameraPawn::OnLookPitch(float AxisValue)
{
	if (bRotateEnabled())
	{
		PitchDeg = FMath::Clamp(PitchDeg + AxisValue * 0.35, -89.5, 89.5);
	}
}

void AOrbitCameraPawn::OnZoom(float AxisValue)
{
	if (!FMath::IsNearlyZero(AxisValue))
	{
		DistanceUU = FMath::Clamp(DistanceUU * FMath::Pow(0.85, AxisValue), 20.0, 400000.0);
	}
}

void AOrbitCameraPawn::OnMoveForward(float AxisValue)
{
	MoveInput.Y += AxisValue;
}

void AOrbitCameraPawn::OnMoveRight(float AxisValue)
{
	MoveInput.X += AxisValue;
}

void AOrbitCameraPawn::OnMoveUp(float AxisValue)
{
	VerticalInput += AxisValue;
}

void AOrbitCameraPawn::OnFocusNext()
{
	if (!Sim.IsValid())
	{
		return;
	}
	const int32 NumTargets = 1 + Sim->GetSystem().Bodies.Num();
	FocusIndex = (FocusIndex + 1) % NumTargets;
	FocusOffsetUU = FVector::ZeroVector;
	DistanceUU = DefaultDistanceForFocus();
}

void AOrbitCameraPawn::OnFocusSpacecraft()
{
	if (!Sim.IsValid())
	{
		return;
	}
	FocusIndex = 0;
	FocusOffsetUU = FVector::ZeroVector;
	DistanceUU = 220.0;
}

void AOrbitCameraPawn::OnResetView()
{
	FocusOffsetUU = FVector::ZeroVector;
	DistanceUU = DefaultDistanceForFocus();
}

void AOrbitCameraPawn::OnReplan()
{
	if (Sim.IsValid())
	{
		Sim->ReplanMission();
	}
}

void AOrbitCameraPawn::OnToggleDebugViz()
{
	if (Sim.IsValid())
	{
		Sim->SetDebugVisualization(!Sim->IsDebugVisualizationEnabled());
	}
}

void AOrbitCameraPawn::SetWarp(int32 WarpIndex)
{
	if (Sim.IsValid())
	{
		Sim->SetWarpIndex(WarpIndex);
	}
}

FVector AOrbitCameraPawn::ComputeFocusPosition() const
{
	if (!Sim.IsValid())
	{
		return FVector::ZeroVector;
	}

	if (FocusIndex == 0)
	{
		FVector Position;
		if (Sim->GetSpacecraftVisualPosition(Position))
		{
			return Position;
		}
		return FVector::ZeroVector;
	}

	const TArray<FCelestialBodyDesc>& Bodies = Sim->GetSystem().Bodies;
	if (FocusIndex - 1 < Bodies.Num())
	{
		const FCelestialBodyDesc& Body = Bodies[FocusIndex - 1];
		FVector Position;
		FVector Velocity;
		if (Sim->GetSystem().GetBodyWorldState(Body.Id, Sim->GetSimTime(), Position, Velocity))
		{
			return Sim->GetVisualScale().ToWorldUnits(Position);
		}
	}
	return FVector::ZeroVector;
}

double AOrbitCameraPawn::DefaultDistanceForFocus() const
{
	if (!Sim.IsValid() || FocusIndex == 0)
	{
		return 400.0;
	}
	const TArray<FCelestialBodyDesc>& Bodies = Sim->GetSystem().Bodies;
	if (FocusIndex - 1 < Bodies.Num())
	{
		const FCelestialBodyDesc& Body = Bodies[FocusIndex - 1];
		return FMath::Max(Sim->GetVisualScale().BodyVisualRadiusUU(Body) * 8.0, 200.0);
	}
	return 400.0;
}
