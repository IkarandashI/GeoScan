#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "Core/Lambert.h"
#include "Core/OrbitMath.h"
#include "Core/PCGRng.h"

namespace
{
	// Real-world constants keep the tests honest against published values.
	constexpr double SunMu = 1.32712440018e20;   // m^3/s^2
	constexpr double EarthMu = 3.986004418e14;   // m^3/s^2
	constexpr double EarthSunDistance = 1.495978707e11; // 1 AU, m

	bool VectorsNearlyEqual(const FVector& A, const FVector& B, double RelativeTolerance)
	{
		const double Scale = FMath::Max(1.0, FMath::Max(A.Size(), B.Size()));
		return (A - B).Size() <= RelativeTolerance * Scale;
	}
}

// ---------------------------------------------------------------- PCG32

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPCG32DeterminismTest, "GeoScan.Core.PCG32.Determinism",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FPCG32DeterminismTest::RunTest(const FString&)
{
	FPCG32 First(42);
	FPCG32 Second(42);
	for (int32 i = 0; i < 1000; ++i)
	{
		if (First.NextUInt32() != Second.NextUInt32())
		{
			AddError(TEXT("Same seed produced different sequences"));
			return false;
		}
	}

	// Different seeds must (practically certainly) diverge.
	FPCG32 SeedA(42);
	FPCG32 SeedB(43);
	bool bAnyDifference = false;
	for (int32 i = 0; i < 100; ++i)
	{
		bAnyDifference |= SeedA.NextUInt32() != SeedB.NextUInt32();
	}
	TestTrue("Different seeds diverge", bAnyDifference);

	// Range contract.
	FPCG32 Rng(7);
	bool bRangeOk = true;
	for (int32 i = 0; i < 1000; ++i)
	{
		const double V = Rng.Range(5.0, 10.0);
		bRangeOk &= (V >= 5.0 && V < 10.0);
	}
	TestTrue("Range stays within [Min, Max)", bRangeOk);

	return true;
}

// ---------------------------------------------------------------- Circular orbits

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FCircularOrbitStateTest, "GeoScan.Core.OrbitMath.CircularState",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FCircularOrbitStateTest::RunTest(const FString&)
{
	FVector Position;
	FVector Velocity;

	OrbitMath::CircularOrbitState(SunMu, EarthSunDistance, 0.0, 0.0, Position, Velocity);
	TestTrue("Radius at t=0", FMath::IsNearlyEqual(Position.Size(), EarthSunDistance, 1.0));
	// Earth's orbital speed around the Sun is ~29.78 km/s.
	TestTrue("Speed ~29.8 km/s", FMath::IsNearlyEqual(Velocity.Size(), 29784.0, 20.0));

	// A quarter period later the body must sit at +90 degrees.
	const double Period = OrbitMath::OrbitalPeriod(SunMu, EarthSunDistance);
	const double Speed = OrbitMath::CircularOrbitSpeed(SunMu, EarthSunDistance);

	OrbitMath::CircularOrbitState(SunMu, EarthSunDistance, 0.0, 0.25 * Period, Position, Velocity);
	TestTrue("Quarter-period position", VectorsNearlyEqual(Position, FVector(0.0, EarthSunDistance, 0.0), 1e-9));
	TestTrue("Quarter-period velocity", VectorsNearlyEqual(Velocity, FVector(-Speed, 0.0, 0.0), 1e-9));

	return true;
}

// ---------------------------------------------------------------- SOI

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FSphereOfInfluenceTest, "GeoScan.Core.OrbitMath.SphereOfInfluence",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FSphereOfInfluenceTest::RunTest(const FString&)
{
	// Earth's SOI is a published value: ~9.24e8 m (~0.0062 AU).
	const double Soi = OrbitMath::SphereOfInfluenceRadius(EarthMu, SunMu, EarthSunDistance);
	TestTrue("Earth SOI ~9.24e8 m", FMath::IsNearlyEqual(Soi, 9.24e8, 0.02e8));
	return true;
}

// ---------------------------------------------------------------- Propagation

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPropagateEllipticTest, "GeoScan.Core.OrbitMath.PropagateElliptic",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FPropagateEllipticTest::RunTest(const FString&)
{
	// 7000 km from Earth's center, 9 km/s tangential: an ellipse (circular speed there is ~7.5 km/s).
	const FVector R0(7.0e6, 0.0, 0.0);
	const FVector V0(0.0, 9.0e3, 0.0);

	const double A = OrbitMath::SemiMajorAxis(R0, V0, EarthMu);
	TestTrue("Elliptic semi-major axis is positive", A > 0.0);

	// After a full period the state must return to the start.
	const double Period = OrbitMath::OrbitalPeriod(EarthMu, A);
	FVector Position;
	FVector Velocity;
	OrbitMath::Propagate(R0, V0, EarthMu, Period, Position, Velocity);
	TestTrue("Full-period position round trip", VectorsNearlyEqual(Position, R0, 1e-8));
	TestTrue("Full-period velocity round trip", VectorsNearlyEqual(Velocity, V0, 1e-8));

	// Invariants at an arbitrary time: energy and angular momentum are conserved.
	OrbitMath::Propagate(R0, V0, EarthMu, 1234.5, Position, Velocity);
	const double Energy0 = OrbitMath::SpecificEnergy(R0, V0, EarthMu);
	const double Energy1 = OrbitMath::SpecificEnergy(Position, Velocity, EarthMu);
	TestTrue("Energy conserved", FMath::IsNearlyEqual(Energy0, Energy1, 1e-6 * FMath::Abs(Energy0)));

	const FVector H0 = FVector::CrossProduct(R0, V0);
	const FVector H1 = FVector::CrossProduct(Position, Velocity);
	TestTrue("Angular momentum conserved", VectorsNearlyEqual(H1, H0, 1e-8));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPropagateHyperbolicTest, "GeoScan.Core.OrbitMath.PropagateHyperbolic",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FPropagateHyperbolicTest::RunTest(const FString&)
{
	// 12 km/s at 7000 km is well above Earth escape speed there (~10.67 km/s).
	const FVector R0(7.0e6, 0.0, 0.0);
	const FVector V0(0.0, 1.2e4, 0.0);
	TestTrue("Orbit is hyperbolic", OrbitMath::SemiMajorAxis(R0, V0, EarthMu) < 0.0);

	FVector Position;
	FVector Velocity;
	OrbitMath::Propagate(R0, V0, EarthMu, 3600.0, Position, Velocity);

	const double Energy0 = OrbitMath::SpecificEnergy(R0, V0, EarthMu);
	const double Energy1 = OrbitMath::SpecificEnergy(Position, Velocity, EarthMu);
	TestTrue("Energy conserved", FMath::IsNearlyEqual(Energy0, Energy1, 1e-6 * FMath::Abs(Energy0)));

	const FVector H0 = FVector::CrossProduct(R0, V0);
	const FVector H1 = FVector::CrossProduct(Position, Velocity);
	TestTrue("Angular momentum conserved", VectorsNearlyEqual(H1, H0, 1e-8));

	// Regression: long-duration hyperbolic propagation. Plain Newton diverges
	// here; the bracketed fallback must keep the result physical.
	const double GiantMu = 2.164e16;
	const FVector GR0(1.5e8, 0.0, 0.0);
	const FVector GV0(0.0, 2.52e4, 0.0);
	const double Vinf = FMath::Sqrt(GV0.SizeSquared() - 2.0 * GiantMu / GR0.Size());
	for (double Days = 10.0; Days <= 300.0; Days *= 3.0)
	{
		FVector GP;
		FVector GV;
		OrbitMath::Propagate(GR0, GV0, GiantMu, Days * 86400.0, GP, GV);
		TestTrue(FString::Printf(TEXT("Hyperbola stays physical at %.0f days"), Days),
			GV.Size() >= Vinf - 1.0 && GV.Size() <= GV0.Size() + 1.0 && FMath::IsFinite(GP.X));
		TestTrue(FString::Printf(TEXT("Hyperbola energy conserved at %.0f days"), Days),
			FMath::IsNearlyEqual(OrbitMath::SpecificEnergy(GP, GV, GiantMu),
				OrbitMath::SpecificEnergy(GR0, GV0, GiantMu), 1e-6 * GV0.SizeSquared()));
	}

	return true;
}

// ---------------------------------------------------------------- Tsiolkovsky

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FTsiolkovskyTest, "GeoScan.Core.OrbitMath.Tsiolkovsky",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FTsiolkovskyTest::RunTest(const FString&)
{
	// 1000 kg total, 500 kg dry, Isp 300 s: budget = 300 * g0 * ln(2).
	const double ExpectedBudget = 300.0 * OrbitMath::G0 * FMath::Loge(2.0);
	const double Budget = OrbitMath::DeltaVBudget(1000.0, 500.0, 300.0);
	TestTrue("Delta-v budget", FMath::IsNearlyEqual(Budget, ExpectedBudget, 1e-6 * ExpectedBudget));

	// Spending the whole budget must consume exactly all fuel.
	const double Burned = OrbitMath::FuelBurned(1000.0, Budget, 300.0);
	TestTrue("Whole budget burns all fuel", FMath::IsNearlyEqual(Burned, 500.0, 1e-6));

	TestTrue("Zero delta-v burns nothing", OrbitMath::FuelBurned(1000.0, 0.0, 300.0) == 0.0);
	return true;
}

// ---------------------------------------------------------------- Lambert round trips

// Helper: propagate a known orbit for Dt, then Lambert must reconstruct the
// departure/arrival velocities that produced it.
static bool LambertRoundTrip(FAutomationTestBase& Test, const FVector& R1, const FVector& V1,
	double Dt, double Tolerance)
{
	FVector R2;
	FVector V2Expected;
	OrbitMath::Propagate(R1, V1, SunMu, Dt, R2, V2Expected);

	const bool bShortWay = FVector::CrossProduct(R1, R2).Z > 0.0;
	FVector V1Out;
	FVector V2Out;
	const bool bSolved = Lambert::Solve(R1, R2, SunMu, Dt, bShortWay, V1Out, V2Out);
	if (!Test.TestTrue("Lambert solved", bSolved))
	{
		return false;
	}
	Test.TestTrue("Departure velocity reconstructed", VectorsNearlyEqual(V1Out, V1, Tolerance));
	Test.TestTrue("Arrival velocity reconstructed", VectorsNearlyEqual(V2Out, V2Expected, Tolerance));
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLambertEllipticTest, "GeoScan.Core.Lambert.RoundTripElliptic",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FLambertEllipticTest::RunTest(const FString&)
{
	// 1 AU out, 35 km/s tangential: an elliptic heliocentric orbit (~92.6 day leg).
	LambertRoundTrip(*this, FVector(EarthSunDistance, 0.0, 0.0), FVector(0.0, 3.5e4, 0.0), 8.0e6, 1e-8);
	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FLambertHyperbolicTest, "GeoScan.Core.Lambert.RoundTripHyperbolic",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FLambertHyperbolicTest::RunTest(const FString&)
{
	// 45 km/s at 1 AU exceeds solar escape speed there (~42.1 km/s): hyperbolic leg.
	LambertRoundTrip(*this, FVector(EarthSunDistance, 0.0, 0.0), FVector(0.0, 4.5e4, 0.0), 3.0e6, 1e-8);
	return true;
}

#endif // WITH_DEV_AUTOMATION_TESTS
