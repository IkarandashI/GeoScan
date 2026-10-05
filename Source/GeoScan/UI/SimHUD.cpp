#include "UI/SimHUD.h"

#include "Engine/Canvas.h"
#include "Engine/Engine.h"
#include "Sim/SpacecraftState.h"
#include "Sim/VisualScale.h"
#include "Systems/SimulationSubsystem.h"

void ASimHUD::DrawHUD()
{
	Super::DrawHUD();

	USimulationSubsystem* Sim = GetWorld() ? GetWorld()->GetSubsystem<USimulationSubsystem>() : nullptr;
	if (Sim == nullptr)
	{
		return;
	}

	UFont* Font = GEngine ? GEngine->GetSmallFont() : nullptr;
	if (Font == nullptr)
	{
		return;
	}

	const FLinearColor White(0.9f, 0.9f, 0.95f, 1.0f);
	const FLinearColor Accent(0.25f, 0.8f, 1.0f, 1.0f);
	const FLinearColor Dimmed(0.6f, 0.6f, 0.65f, 1.0f);

	double X = 30.0;
	double Y = 30.0;
	const double LineHeight = 18.0;
	auto DrawLineOfText = [&](const FString& Text, const FLinearColor& Color)
	{
		DrawText(Text, Color, X, Y, Font, 1.0f, false);
		Y += LineHeight;
	};

	if (Sim->GetStatus() == EMissionStatus::Idle)
	{
		DrawLineOfText(TEXT("GeoScan: no scenario loaded"), White);
		DrawLineOfText(TEXT("Set a valid UScenarioConfig (see Content/Scenarios) and press Play."), Dimmed);
		return;
	}

	const double Au = FVisualScaleSettings::AU;
	const double SimTime = Sim->GetSimTime();
	const int32 WarpIndex = Sim->GetWarpIndex();
	const FString WarpText = WarpIndex == 0
		? TEXT("PAUSED")
		: FString::Printf(TEXT("%.0gx"), Sim->GetWarpLevels()[WarpIndex]);

	// World state and relative-to-dominant-body figures.
	const FSpacecraftState& Spacecraft = Sim->GetSpacecraft();
	FVector WorldPosition;
	FVector WorldVelocity;
	Sim->GetSpacecraftWorldState(WorldPosition, WorldVelocity);

	const FCelestialBodyDesc* Dominant = Sim->GetSystem().FindBody(Spacecraft.DominantBodyId);
	double RelativeSpeedKmS = 0.0;
	double AltitudeM = 0.0;
	if (Dominant)
	{
		FVector BodyPosition;
		FVector BodyVelocity;
		Sim->GetSystem().GetBodyWorldState(Dominant->Id, SimTime, BodyPosition, BodyVelocity);
		RelativeSpeedKmS = (WorldVelocity - BodyVelocity).Size() / 1000.0;
		AltitudeM = (WorldPosition - BodyPosition).Size() - Dominant->Radius;
	}

	const double StarDistanceAU = WorldPosition.Size() / Au;
	const double BoundaryAU = Sim->GetSystemBoundary() / Au;

	DrawLineOfText(TEXT("GeoScan - Space Navigation Simulation"), Accent);
	DrawLineOfText(FString::Printf(TEXT("Mission time: %.2f y (%.1f days)   Warp: %s"),
		SimTime / 31557600.0, SimTime / 86400.0, *WarpText), White);
	DrawLineOfText(FString::Printf(TEXT("Dominant body: %s   Rel speed: %.2f km/s   Altitude: %.3e m"),
		Dominant ? *Dominant->Id.ToString() : TEXT("?"), RelativeSpeedKmS, AltitudeM), White);
	DrawLineOfText(FString::Printf(TEXT("Mass: %.0f kg (fuel %.0f)   dV budget: %.2f km/s"),
		Spacecraft.TotalMass(), Spacecraft.FuelMass, Spacecraft.DeltaVRemaining() / 1000.0), White);
	DrawLineOfText(FString::Printf(TEXT("Star distance: %.3f AU / boundary %.1f AU (%.2f%%)"),
		StarDistanceAU, BoundaryAU, BoundaryAU > 0.0 ? 100.0 * StarDistanceAU / BoundaryAU : 0.0), White);

	// Route + plan metrics.
	{
		FString RouteText;
		for (const FName& Body : Sim->GetRouteBodies())
		{
			RouteText += FString::Printf(TEXT("%s "), *Body.ToString());
		}
		const FMissionPlan& Plan = Sim->GetLoadedPlan();
		if (!RouteText.IsEmpty())
		{
			RouteText += TEXT("-> boundary");
		}
		else if (Plan.IsValid())
		{
			RouteText = TEXT("direct escape");
		}
		if (!RouteText.IsEmpty())
		{
			DrawLineOfText(FString::Printf(TEXT("Route: %s | plan dV %.2f km/s, %.2f y, risk %.2f"),
				*RouteText, Plan.TotalDeltaV / 1000.0, Plan.FlightTime / 31557600.0, Plan.RiskScore), White);
		}
	}

	// Autopilot plan status.
	FManeuverEvent NextManeuver;
	if (Sim->GetPendingManeuverCount() > 0 && Sim->GetNextManeuver(NextManeuver))
	{
		DrawLineOfText(FString::Printf(TEXT("Plan: %d done / %d pending | next: %s in %.1f d (dV %.2f km/s)"),
			Sim->GetExecutedManeuverCount(), Sim->GetPendingManeuverCount(),
			NextManeuver.Note != NAME_None ? *NextManeuver.Note.ToString() : TEXT("burn"),
			(NextManeuver.Time - SimTime) / 86400.0,
			NextManeuver.DeltaV.Size() / 1000.0), White);
	}
	else if (Sim->GetExecutedManeuverCount() > 0)
	{
		DrawLineOfText(FString::Printf(TEXT("Plan: all %d maneuvers executed"), Sim->GetExecutedManeuverCount()), White);
	}

	if (Sim->GetStatus() == EMissionStatus::Success)
	{
		DrawLineOfText(FString::Printf(TEXT(">>> MISSION SUCCESS at %.2f years <<<"), SimTime / 31557600.0), Accent);
	}
	else if (Sim->GetStatus() != EMissionStatus::Running && Sim->GetStatus() != EMissionStatus::Idle)
	{
		DrawLineOfText(FString::Printf(TEXT(">>> MISSION FAILED: %s <<<"), *Sim->GetFailureReason()), FLinearColor(1.0f, 0.35f, 0.3f, 1.0f));
	}

	Y += 6.0;
	DrawLineOfText(TEXT("[0-8] time warp (0=pause)   [C] focus spacecraft   [F] next focus   [R] reset view   [P] replan   [V] debug viz"), Dimmed);
	DrawLineOfText(TEXT("[RMB+mouse] rotate   [wheel] zoom   [WASD/QE] pan"), Dimmed);

	// World-space labels: every body and the spacecraft are tagged on screen.
	if (Canvas && Sim->GetStatus() != EMissionStatus::Idle)
	{
		const FVisualScaleSettings& Scale = Sim->GetVisualScale();
		auto DrawLabel = [&](const FVector& WorldUU, const FString& Text, const FLinearColor& Color)
		{
			const FVector Projected = Canvas->Project(WorldUU);
			if (Projected.X < 0.0 || Projected.Y < 0.0
				|| Projected.X > Canvas->SizeX || Projected.Y > Canvas->SizeY)
			{
				return;
			}
			DrawText(Text, Color, Projected.X + 10.0f, Projected.Y - 7.0f, Font, 1.0f, false);
		};

		for (const FCelestialBodyDesc& Body : Sim->GetSystem().Bodies)
		{
			FVector Position;
			FVector Velocity;
			if (Sim->GetSystem().GetBodyWorldState(Body.Id, SimTime, Position, Velocity))
			{
				DrawLabel(Scale.ToWorldUnits(Position), Body.Id.ToString(), Dimmed);
			}
		}

		FVector SpacecraftVisualPosition;
		if (Sim->GetSpacecraftVisualPosition(SpacecraftVisualPosition))
		{
			DrawLabel(SpacecraftVisualPosition, TEXT("< spacecraft"), Accent);
		}
	}
}
