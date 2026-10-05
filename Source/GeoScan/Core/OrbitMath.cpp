#include "Core/OrbitMath.h"

namespace OrbitMath
{

double StumpffC(double Z)
{
	if (Z > 1e-8)
	{
		const double SqrtZ = FMath::Sqrt(Z);
		return (1.0 - FMath::Cos(SqrtZ)) / Z;
	}
	if (Z < -1e-8)
	{
		const double SqrtZ = FMath::Sqrt(-Z);
		return (FMath::Cosh(SqrtZ) - 1.0) / (-Z);
	}
	// Series expansion near zero: the closed forms lose all precision there.
	return 0.5 - Z / 24.0 + Z * Z / 720.0;
}

double StumpffS(double Z)
{
	if (Z > 1e-8)
	{
		const double SqrtZ = FMath::Sqrt(Z);
		return (SqrtZ - FMath::Sin(SqrtZ)) / (SqrtZ * SqrtZ * SqrtZ);
	}
	if (Z < -1e-8)
	{
		const double SqrtZ = FMath::Sqrt(-Z);
		return (FMath::Sinh(SqrtZ) - SqrtZ) / (SqrtZ * SqrtZ * SqrtZ);
	}
	return 1.0 / 6.0 - Z / 120.0 + Z * Z / 5040.0;
}

double CircularOrbitSpeed(double Mu, double Radius)
{
	return FMath::Sqrt(Mu / Radius);
}

double OrbitalPeriod(double Mu, double SemiMajorAxis)
{
	return 2.0 * UE_DOUBLE_PI * FMath::Sqrt(SemiMajorAxis * SemiMajorAxis * SemiMajorAxis / Mu);
}

void CircularOrbitState(double Mu, double Radius, double InitialPhase, double Time,
	FVector& OutPosition, FVector& OutVelocity)
{
	const double AngularRate = FMath::Sqrt(Mu / (Radius * Radius * Radius));
	const double Theta = InitialPhase + AngularRate * Time;
	const double Speed = CircularOrbitSpeed(Mu, Radius);

	OutPosition = FVector(FMath::Cos(Theta) * Radius, FMath::Sin(Theta) * Radius, 0.0);
	OutVelocity = FVector(-FMath::Sin(Theta) * Speed, FMath::Cos(Theta) * Speed, 0.0);
}

double SphereOfInfluenceRadius(double MuBody, double MuParent, double OrbitRadius)
{
	return OrbitRadius * FMath::Pow(MuBody / MuParent, 0.4);
}

double SpecificEnergy(const FVector& Position, const FVector& Velocity, double Mu)
{
	return 0.5 * Velocity.SizeSquared() - Mu / Position.Size();
}

double SemiMajorAxis(const FVector& Position, const FVector& Velocity, double Mu)
{
	// a = -Mu / (2 * Energy); positive for ellipses, negative for hyperbolas.
	return -Mu / (2.0 * SpecificEnergy(Position, Velocity, Mu));
}

void Propagate(const FVector& Position0, const FVector& Velocity0, double Mu, double DeltaTime,
	FVector& OutPosition, FVector& OutVelocity)
{
	const double R0 = Position0.Size();
	const double V0 = Velocity0.Size();
	const double RadialRate = FVector::DotProduct(Position0, Velocity0) / R0;
	const double Alpha = 2.0 / R0 - V0 * V0 / Mu; // = 1 / a
	const double SqrtMu = FMath::Sqrt(Mu);

	// Universal Kepler equation F(chi) = 0; F is strictly increasing in chi,
	// which makes a bracketed fallback bulletproof when Newton misbehaves
	// (it does, on long-duration hyperbolas).
	auto UniversalKepler = [&](double Chi) -> double
	{
		const double Z = Alpha * Chi * Chi;
		const double C = StumpffC(Z);
		const double S = StumpffS(Z);
		return R0 * RadialRate / SqrtMu * Chi * Chi * C
			+ (1.0 - Alpha * R0) * Chi * Chi * Chi * S
			+ R0 * Chi - SqrtMu * DeltaTime;
	};

	// Initial guess for the universal anomaly (Vallado Alg. 8).
	double Chi = 0.0;
	if (Alpha > 1e-12)
	{
		Chi = SqrtMu * DeltaTime * Alpha; // elliptic: ~ sqrt(a) * mean anomaly
	}
	else if (Alpha < -1e-12)
	{
		Chi = FMath::Sign(DeltaTime) * FMath::Sqrt(-1.0 / Alpha); // hyperbolic
	}
	else
	{
		Chi = SqrtMu * DeltaTime / R0; // near-parabolic
	}

	// Fast path: Newton.
	bool bConverged = false;
	constexpr int32 MaxNewtonIterations = 100;
	for (int32 Iteration = 0; Iteration < MaxNewtonIterations; ++Iteration)
	{
		const double Z = Alpha * Chi * Chi;
		const double C = StumpffC(Z);
		const double S = StumpffS(Z);

		const double Residual =
			R0 * RadialRate / SqrtMu * Chi * Chi * C +
			(1.0 - Alpha * R0) * Chi * Chi * Chi * S +
			R0 * Chi - SqrtMu * DeltaTime;

		const double Derivative =
			R0 * RadialRate / SqrtMu * Chi * (1.0 - Z * S) +
			(1.0 - Alpha * R0) * Chi * Chi * C +
			R0;

		const double DeltaChi = Residual / Derivative;
		Chi -= DeltaChi;

		if (FMath::Abs(DeltaChi) <= 1e-12 * FMath::Max(1.0, FMath::Abs(Chi)))
		{
			bConverged = FMath::Abs(UniversalKepler(Chi)) <= 1e-9 * SqrtMu * FMath::Abs(DeltaTime);
			break;
		}
	}

	// Fallback: bracket and bisect — guaranteed, since F is monotone in chi.
	if (!bConverged)
	{
		const double TargetScale = SqrtMu * FMath::Abs(DeltaTime);
		double Low = 0.0;
		double High = FMath::Max(Chi, 1.0) * FMath::Sign(DeltaTime);
		if (High == 0.0)
		{
			High = 1.0;
		}
		for (int32 Expansion = 0; Expansion < 200; ++Expansion)
		{
			if (UniversalKepler(High) * FMath::Sign(DeltaTime) > 0.0)
			{
				break;
			}
			Low = High;
			High *= 2.0;
		}
		for (int32 Iteration = 0; Iteration < 100; ++Iteration)
		{
			const double Mid = 0.5 * (Low + High);
			if ((UniversalKepler(Mid) < 0.0) == (DeltaTime > 0.0))
			{
				Low = Mid;
			}
			else
			{
				High = Mid;
			}
		}
		Chi = 0.5 * (Low + High);
	}

	const double Z = Alpha * Chi * Chi;
	const double C = StumpffC(Z);
	const double S = StumpffS(Z);

	const double F = 1.0 - Chi * Chi / R0 * C;
	const double G = DeltaTime - Chi * Chi * Chi / SqrtMu * S;

	FVector Position = F * Position0 + G * Velocity0;
	const double R = Position.Size();

	const double FDot = SqrtMu / (R * R0) * Chi * (Z * S - 1.0);
	const double GDot = 1.0 - Chi * Chi / R * C;

	OutPosition = Position;
	OutVelocity = FDot * Position0 + GDot * Velocity0;
}

double FuelBurned(double MassTotal0, double DeltaV, double Isp)
{
	return MassTotal0 * (1.0 - FMath::Exp(-DeltaV / (Isp * G0)));
}

double DeltaVBudget(double MassTotal0, double MassDry, double Isp)
{
	return Isp * G0 * FMath::Loge(MassTotal0 / MassDry);
}

} // namespace OrbitMath
