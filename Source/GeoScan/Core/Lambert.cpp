#include "Core/Lambert.h"
#include "Core/OrbitMath.h"

namespace Lambert
{

bool Solve(const FVector& Position1, const FVector& Position2, double Mu, double DeltaTime, bool bShortWay,
	FVector& OutVelocity1, FVector& OutVelocity2)
{
	if (DeltaTime <= 0.0)
	{
		return false;
	}

	const double R1 = Position1.Size();
	const double R2 = Position2.Size();
	const double CosDnu = FMath::Clamp(FVector::DotProduct(Position1, Position2) / (R1 * R2), -1.0, 1.0);

	const double A = (bShortWay ? 1.0 : -1.0) * FMath::Sqrt(R1 * R2 * (1.0 + CosDnu));
	if (FMath::Abs(A) < 1e-8 * FMath::Max(R1, R2))
	{
		return false; // transfer angle ~180 degrees: singular for this formulation
	}

	const double SqrtMu = FMath::Sqrt(Mu);

	// Bisect on psi = alpha * chi^2 until the induced time of flight matches DeltaTime.
	// Time of flight is monotonically increasing in psi for direct transfers.
	double Psi = 0.0;
	double PsiLow = -4.0 * UE_DOUBLE_PI * UE_DOUBLE_PI;
	double PsiUp = 4.0 * UE_DOUBLE_PI * UE_DOUBLE_PI;

	double C2 = OrbitMath::StumpffC(Psi);
	double C3 = OrbitMath::StumpffS(Psi);
	double Y = 0.0;
	double Chi = 0.0;
	bool bConverged = false;

	constexpr int32 MaxIterations = 200;
	for (int32 Iteration = 0; Iteration < MaxIterations; ++Iteration)
	{
		if (C2 < 1e-10)
		{
			return false; // psi hit a Stumpff-C zero — would need the multi-revolution branch
		}

		Y = R1 + R2 + A * (Psi * C3 - 1.0) / FMath::Sqrt(C2);

		// For the short way Y must stay positive; nudge psi up until it is (Vallado).
		if (A > 0.0 && Y < 0.0)
		{
			int32 RaiseGuard = 0;
			while (Y < 0.0 && ++RaiseGuard < 1000)
			{
				Psi += 0.1;
				C2 = OrbitMath::StumpffC(Psi);
				C3 = OrbitMath::StumpffS(Psi);
				Y = R1 + R2 + A * (Psi * C3 - 1.0) / FMath::Sqrt(C2);
			}
			if (Y < 0.0)
			{
				return false;
			}
			PsiLow = Psi;
		}

		Chi = FMath::Sqrt(Y / C2);
		const double TimeOfFlight = (Chi * Chi * Chi * C3 + A * FMath::Sqrt(Y)) / SqrtMu;

		if (TimeOfFlight <= DeltaTime)
		{
			PsiLow = Psi;
		}
		else
		{
			PsiUp = Psi;
		}

		if (FMath::Abs(TimeOfFlight - DeltaTime) <= 1e-9 * FMath::Abs(DeltaTime))
		{
			bConverged = true;
			break;
		}

		Psi = 0.5 * (PsiLow + PsiUp);
		C2 = OrbitMath::StumpffC(Psi);
		C3 = OrbitMath::StumpffS(Psi);
	}

	if (!bConverged)
	{
		return false;
	}

	const double F = 1.0 - Y / R1;
	const double G = A * FMath::Sqrt(Y / Mu);
	const double GDot = 1.0 - Y / R2;

	OutVelocity1 = (Position2 - F * Position1) / G;
	OutVelocity2 = (GDot * Position2 - Position1) / G;
	return true;
}

} // namespace Lambert
