#pragma once

#include "CoreMinimal.h"
#include "Gen/SystemTypes.h"

/**
 * Physics runs in honest meters (double); rendering compresses distances so that
 * a planetary system is readable on screen (decision Q4). Pure mapping, no UObject.
 */
struct FVisualScaleSettings
{
	/** How many UE world units one astronomical unit occupies on screen. */
	double WorldUnitsPerAU = 2500.0;

	/** Bodies are never drawn smaller than this — true radii are sub-pixel at system scale. */
	double MinPlanetVisualRadiusUU = 30.0;
	double MinStarVisualRadiusUU = 150.0;
	double SpacecraftVisualRadiusUU = 8.0;

	static constexpr double AU = 1.495978707e11; // m

	double ScaleFactor() const { return WorldUnitsPerAU / AU; }

	FVector ToWorldUnits(const FVector& Meters) const { return Meters * ScaleFactor(); }

	double BodyVisualRadiusUU(const FCelestialBodyDesc& Body) const
	{
		const double MinUU = Body.ParentId == NAME_None ? MinStarVisualRadiusUU : MinPlanetVisualRadiusUU;
		return FMath::Max(Body.Radius * ScaleFactor(), MinUU);
	}

	/** SOI ring radius: the global scale, but never smaller than the oversized ball. */
	double VisualSoiRadiusUU(const FCelestialBodyDesc& Body) const
	{
		return FMath::Max(Body.SoiRadius * ScaleFactor(), BodyVisualRadiusUU(Body) * 1.3);
	}

	/**
	 * Visual position of a point RelMeters from a body's center. Bodies are drawn
	 * oversized, so close-range motion is remapped: just above the surface it sits
	 * outside the ball, fading to the honest global scale at the SOI edge —
	 * continuous with interplanetary rendering (tiny jump possible for planets
	 * whose whole SOI is smaller than their ball; subpixel at system zoom).
	 */
	FVector ToWorldUnitsAroundBody(const FCelestialBodyDesc& Body, const FVector& RelMeters) const
	{
		const double Rel = RelMeters.Size();
		if (Body.ParentId == NAME_None || Body.SoiRadius <= Body.Radius || Rel <= Body.Radius)
		{
			const FVector Unit = Rel > 0.0 ? RelMeters / Rel : FVector::XAxisVector;
			return Unit * FMath::Max(BodyVisualRadiusUU(Body) * 1.15, Rel * ScaleFactor());
		}

		const double Fade = FMath::Clamp((Rel - Body.Radius) / (Body.SoiRadius - Body.Radius), 0.0, 1.0);
		const double NearUU = BodyVisualRadiusUU(Body) * 1.15;
		const double RVis = FMath::Lerp(NearUU, VisualSoiRadiusUU(Body), Fade);
		return RelMeters / Rel * RVis;
	}

	/** World (star-centered) visual position of something near Body. */
	FVector ToVisualAroundBody(const FCelestialBodyDesc& Body, const FVector& BodyWorldMeters, const FVector& WorldMeters) const
	{
		return ToWorldUnits(BodyWorldMeters) + ToWorldUnitsAroundBody(Body, WorldMeters - BodyWorldMeters);
	}
};
