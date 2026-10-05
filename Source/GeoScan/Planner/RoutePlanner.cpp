#include "Planner/RoutePlanner.h"

#include "Core/Lambert.h"
#include "Core/OrbitMath.h"
#include "Planner/RouteCriteria.h"
#include "Sim/MissionRunner.h"

namespace
{
	constexpr double DaySeconds = 86400.0;

	struct FSearchNode
	{
		FName BodyId = NAME_None;   // body whose SOI we just entered
		double ArrivalTime = 0.0;
		FVector VinInf = FVector::ZeroVector; // arrival hyperbolic excess, body frame
		double TotalDeltaV = 0.0;
		double FuelUsed = 0.0;
		double MassTotal = 0.0;
		double MaxRisk = 0.0;
		TArray<FManeuverEvent> Maneuvers;
		TArray<FName> Visits;
		TArray<struct FEdgeRecord> Edges;

		// Heliocentric (replan) starts store their first corrective burn here;
		// parking-orbit starts recompute the departure burn from parking geometry.
		bool bHeliocentricStart = false;
		FManeuverEvent FirstLegBurn;
	};

	struct FEdgeRecord
	{
		FName FromBody;
		FName ToBody;               // NAME_None = escape to boundary
		double DepartureTime = 0.0;
		double ArrivalTime = 0.0;
		FVector DepVinf = FVector::ZeroVector;
		FVector ArrVinf = FVector::ZeroVector;
		double PeriapsisRadius = 0.0;
	};

	struct FCandidate
	{
		double Cost = 0.0;
		double TotalDeltaV = 0.0;
		double FlightTime = 0.0;
		double RiskScore = 0.0;
		TArray<FManeuverEvent> Maneuvers;
		TArray<FName> RouteBodies;
		TArray<FEdgeRecord> Edges;
		bool bHeliocentricStart = false;
		FManeuverEvent FirstLegBurn;
	};

	FVector RotateAroundZ(const FVector& V, double AngleRad)
	{
		const double C = FMath::Cos(AngleRad);
		const double S = FMath::Sin(AngleRad);
		return FVector(C * V.X - S * V.Y, S * V.X + C * V.Y, V.Z);
	}

	// Parking-orbit geometry, extracted once from the initial state.
	struct FParkingOrbit
	{
		const FCelestialBodyDesc* Body = nullptr;
		double Radius = 0.0;
		double AngularRate = 0.0;
		double PhaseAtEpoch = 0.0;
		double Epoch = 0.0;
		double VCirc = 0.0;
		FVector P;
		FVector Q;

		FVector RelativePositionAt(double Time) const
		{
			const double Phi = PhaseAtEpoch + AngularRate * (Time - Epoch);
			return Radius * (FMath::Cos(Phi) * P + FMath::Sin(Phi) * Q);
		}

		FVector WorldPositionAt(const FPlanetarySystem& System, double Time) const
		{
			FVector BodyPos;
			FVector BodyVel;
			System.GetBodyWorldState(Body->Id, Time, BodyPos, BodyVel);
			return BodyPos + RelativePositionAt(Time);
		}
	};

	FParkingOrbit MakeParkingOrbit(const FPlanetarySystem& System, const FSpacecraftState& Initial)
	{
		FParkingOrbit Parking;
		Parking.Body = System.FindBody(Initial.DominantBodyId);
		Parking.Radius = Initial.Position.Size();
		Parking.VCirc = Initial.Velocity.Size();
		Parking.Epoch = Initial.Epoch;
		Parking.AngularRate = FMath::Sqrt(Parking.Body->Mu / (Parking.Radius * Parking.Radius * Parking.Radius));
		FCelestialBodyDesc::GetOrbitBasis(Parking.Body->Inclination, Parking.Body->LAN, Parking.P, Parking.Q);
		Parking.PhaseAtEpoch = FMath::Atan2(
			FVector::DotProduct(Initial.Position, Parking.Q), FVector::DotProduct(Initial.Position, Parking.P));
		return Parking;
	}

	/** Earliest time >= Earliest at which the parking velocity points along VinfDir. */
	double ProgradeBurnTime(const FParkingOrbit& Parking, const FVector& VinfDir, double Earliest)
	{
		const double Alpha = FMath::Atan2(
			FVector::DotProduct(VinfDir, Parking.Q), FVector::DotProduct(VinfDir, Parking.P));
		double Rel = Alpha - 0.5 * UE_DOUBLE_PI - Parking.PhaseAtEpoch - Parking.AngularRate * (Earliest - Parking.Epoch);
		Rel = FMath::Fmod(Rel, 2.0 * UE_DOUBLE_PI);
		if (Rel < 0.0)
		{
			Rel += 2.0 * UE_DOUBLE_PI;
		}
		return Earliest + Rel / Parking.AngularRate;
	}

	double MaxTurnAngle(double Mu, double SafePeriapsis, double Vinf)
	{
		const double Ecc = 1.0 + SafePeriapsis * Vinf * Vinf / Mu;
		return 2.0 * FMath::Asin(FMath::Clamp(1.0 / Ecc, 0.0, 1.0));
	}

	/** Periapsis radius that produces the requested turn at the given hyperbolic speed. */
	double PeriapsisForTurn(double TurnAngleRad, double Mu, double Vinf)
	{
		if (TurnAngleRad <= 1e-6)
		{
			return 0.0; // degenerate: no bend needed
		}
		const double Ecc = 1.0 / FMath::Sin(0.5 * TurnAngleRad);
		return (Ecc - 1.0) * Mu / (Vinf * Vinf);
	}

	/** First time the conic reaches TargetR (coarse scan + bisection); false if never within MaxT. */
	bool TimeToReachDistance(const FVector& R0, const FVector& V0, double Mu, double TargetR, double MaxT, double& OutTime)
	{
		if (R0.Size() >= TargetR)
		{
			OutTime = 0.0;
			return true;
		}
		const double Energy = OrbitMath::SpecificEnergy(R0, V0, Mu);
		if (Energy < 0.0)
		{
			const double A = -Mu / (2.0 * Energy);
			const double H = FVector::CrossProduct(R0, V0).Size();
			const double Ecc = FMath::Sqrt(FMath::Max(0.0, 1.0 + 2.0 * Energy * H * H / (Mu * Mu)));
			if (A * (1.0 + Ecc) < TargetR)
			{
				return false; // bound orbit that never reaches the boundary
			}
		}

		constexpr int32 CoarseSteps = 96;
		double PrevT = 0.0;
		for (int32 Step = 1; Step <= CoarseSteps; ++Step)
		{
			const double T = MaxT * Step / CoarseSteps;
			FVector Rt;
			FVector Vt;
			OrbitMath::Propagate(R0, V0, Mu, T, Rt, Vt);
			if (Rt.Size() >= TargetR)
			{
				double Low = PrevT;
				double High = T;
				for (int32 Iteration = 0; Iteration < 40; ++Iteration)
				{
					const double Mid = 0.5 * (Low + High);
					OrbitMath::Propagate(R0, V0, Mu, Mid, Rt, Vt);
					if (Rt.Size() >= TargetR)
					{
						High = Mid;
					}
					else
					{
						Low = Mid;
					}
				}
				OutTime = High;
				return true;
			}
			PrevT = T;
		}
		return false;
	}

	bool SolveLambertPrograde(const FVector& From, const FVector& To, double Mu, double Tof, FVector& OutV1, FVector& OutV2)
	{
		const bool bShortWay = FVector::CrossProduct(From, To).Z > 0.0;
		return Lambert::Solve(From, To, Mu, Tof, bShortWay, OutV1, OutV2);
	}
}

namespace RoutePlanner
{

FPlanningResult PlanRoute(const FPlanRequest& Request)
{
	FPlanningResult Result;
	if (Request.Criteria.IsEmpty())
	{
		Result.FailReason = TEXT("No optimization criteria configured");
		return Result;
	}

	const FPlanetarySystem& System = Request.System;
	const FSpacecraftState& Init = Request.InitialState;
	const FPlannerSearchParams& Search = Request.Search;

	const FCelestialBodyDesc* Star = &System.GetStar();
	// Replanning mid-flight starts from a heliocentric state: the parking-orbit
	// departure machinery is replaced by a corrective burn off the current conic.
	const bool bHeliocentricStart = Init.DominantBodyId == Star->Id;
	const FParkingOrbit Parking = bHeliocentricStart ? FParkingOrbit() : MakeParkingOrbit(System, Init);
	const FName StartBodyId = bHeliocentricStart ? NAME_None : Parking.Body->Id;
	const double Boundary = System.SystemBoundary;
	const double MaxFlightTime = Request.Limits.MaxFlightTime > 0.0 ? Request.Limits.MaxFlightTime : 1e12;
	const double Budget = Init.DeltaVRemaining();

	auto SpeedOk = [&](double Speed) { return Request.Limits.MaxSpeed <= 0.0 || Speed <= Request.Limits.MaxSpeed; };
	auto EvaluateCost = [&](double TotalDeltaV, double FlightTime, double Risk) -> double
	{
		if (Request.CostEvaluator)
		{
			FRouteMetrics Metrics;
			Metrics.TotalDeltaV = TotalDeltaV;
			Metrics.DeltaVBudget = Budget;
			Metrics.FlightTime = FlightTime;
			Metrics.MaxFlightTime = MaxFlightTime;
			Metrics.RiskScore = Risk;
			return Request.CostEvaluator(Metrics);
		}
		return (TotalDeltaV / Budget + FlightTime / MaxFlightTime + Risk) / 3.0;
	};
	// Deterministic comparator: exact ties broken by time, then dV, then route length.
	auto CandidateOrder = [](const FCandidate& A, const FCandidate& B)
	{
		if (A.Cost != B.Cost) return A.Cost < B.Cost;
		if (A.FlightTime != B.FlightTime) return A.FlightTime < B.FlightTime;
		if (A.TotalDeltaV != B.TotalDeltaV) return A.TotalDeltaV < B.TotalDeltaV;
		return A.RouteBodies.Num() < B.RouteBodies.Num();
	};

	TArray<FName> PlanetIds;
	for (const FCelestialBodyDesc& Body : System.Bodies)
	{
		if (Body.ParentId != NAME_None && Body.Id != StartBodyId)
		{
			PlanetIds.Add(Body.Id);
		}
	}

	TArray<FCandidate> Candidates;
	TArray<FSearchNode> Beam;

	auto LogTof = [&Search](int32 Index) -> double
	{
		const double MinLn = FMath::Loge(Search.MinLegFlightDays * DaySeconds);
		const double MaxLn = FMath::Loge(Search.MaxLegFlightDays * DaySeconds);
		return FMath::Exp(FMath::Lerp(MinLn, MaxLn, static_cast<double>(Index) / FMath::Max(1, Search.FlightTimeSteps - 1)));
	};

	// Cumulative fuel chain: returns the burn for NewDeltaV, or -1 if it would exceed the tank.
	auto TryBurn = [&Init](const FSearchNode& Node, double NewDeltaV) -> double
	{
		const double Burn = OrbitMath::FuelBurned(Node.MassTotal, NewDeltaV, Init.Isp);
		return Node.FuelUsed + Burn > Init.FuelMass ? -1.0 : Burn;
	};

	// ------------------------------------------------------------------
	// Level 0: departure legs from the parking orbit (or, for replans from a
	// heliocentric state, corrective burns off the current conic).
	// ------------------------------------------------------------------
	TArray<FSearchNode> Level0;
	if (bHeliocentricStart)
	{
		for (const FName& TargetId : PlanetIds)
		{
			for (int32 CoastIndex = 0; CoastIndex < Search.WaitSteps; ++CoastIndex)
			{
				const double Coast = Search.MaxDepartureWaitDays * DaySeconds * CoastIndex / FMath::Max(1, Search.WaitSteps - 1);
				const double DepartureTime = Init.Epoch + Coast;

				FVector CoastPos;
				FVector CoastVel;
				OrbitMath::Propagate(Init.Position, Init.Velocity, Star->Mu, DepartureTime - Init.Epoch, CoastPos, CoastVel);

				for (int32 TofIndex = 0; TofIndex < Search.FlightTimeSteps; ++TofIndex)
				{
					const double Tof = LogTof(TofIndex);
					const double ArrivalTime = DepartureTime + Tof;
					if (ArrivalTime > MaxFlightTime)
					{
						continue;
					}

					FVector TPos;
					FVector TVel;
					System.GetBodyWorldState(TargetId, ArrivalTime, TPos, TVel);

					FVector VDep;
					FVector VArr;
					if (!SolveLambertPrograde(CoastPos, TPos, Star->Mu, Tof, VDep, VArr))
					{
						continue;
					}
					if (!SpeedOk(VArr.Size()))
					{
						continue;
					}

					FSearchNode Node;
					Node.MassTotal = Init.TotalMass();
					Node.bHeliocentricStart = true;
					Node.FirstLegBurn.Time = DepartureTime;
					Node.FirstLegBurn.DeltaV = VDep - CoastVel;
					Node.FirstLegBurn.Note = FName(TEXT("transfer burn"));
					Node.TotalDeltaV = Node.FirstLegBurn.DeltaV.Size();
					const double Burn = TryBurn(Node, Node.TotalDeltaV);
					if (Node.TotalDeltaV < 1.0 || Burn < 0.0)
					{
						continue;
					}

					Node.BodyId = TargetId;
					Node.ArrivalTime = ArrivalTime;
					Node.VinInf = VArr - TVel;
					Node.FuelUsed = Burn;
					Node.MassTotal -= Burn;
					Node.Visits = { TargetId };

					FEdgeRecord& Edge = Node.Edges.AddDefaulted_GetRef();
					Edge.FromBody = Star->Id;
					Edge.ToBody = TargetId;
					Edge.DepartureTime = DepartureTime;
					Edge.ArrivalTime = ArrivalTime;
					Edge.DepVinf = VDep;
					Edge.ArrVinf = Node.VinInf;

					Level0.Add(Node);
				}
			}
		}
	}
	else
	{
	for (const FName& TargetId : PlanetIds)
	{
		const FCelestialBodyDesc* Target = System.FindBody(TargetId);
		for (int32 WaitIndex = 0; WaitIndex < Search.WaitSteps; ++WaitIndex)
		{
			const double Wait = Search.MaxDepartureWaitDays * DaySeconds * WaitIndex / FMath::Max(1, Search.WaitSteps - 1);
			for (int32 TofIndex = 0; TofIndex < Search.FlightTimeSteps; ++TofIndex)
			{
				const double Tof = LogTof(TofIndex);
				const double ArrivalTime = Wait + Tof;
				if (ArrivalTime > MaxFlightTime)
				{
					continue;
				}

				// Iterate: Lambert fixes the departure v-infinity direction, the burn
				// time must track it (they interact through the exact departure
				// point) so the executor fires exactly along the parking velocity.
				FVector APos;
				FVector AVel;
				FVector TPos;
				FVector TVel;
				System.GetBodyWorldState(TargetId, ArrivalTime, TPos, TVel);
				System.GetBodyWorldState(Parking.Body->Id, Wait, APos, AVel);

				FVector VDep;
				FVector VArr;
				if (!SolveLambertPrograde(APos, TPos, Star->Mu, Tof, VDep, VArr))
				{
					continue;
				}

				FVector DepVinf = VDep - AVel;
				double BurnTime = Wait;
				bool bLegConverged = false;
				for (int32 Pass = 0; Pass < 3; ++Pass)
				{
					const double UGuess = DepVinf.Size();
					if (UGuess < 1.0)
					{
						break;
					}
					const double NewBurnTime = ProgradeBurnTime(Parking, DepVinf / UGuess, Wait);
					const FVector DeparturePos = Parking.WorldPositionAt(System, NewBurnTime);
					const double LegTime = ArrivalTime - NewBurnTime;
					if (LegTime <= 0.0
						|| !SolveLambertPrograde(DeparturePos, TPos, Star->Mu, LegTime, VDep, VArr))
					{
						break;
					}
					System.GetBodyWorldState(Parking.Body->Id, NewBurnTime, APos, AVel);
					DepVinf = VDep - AVel;
					BurnTime = NewBurnTime;
					if (DepVinf.Size() >= 1.0 && SpeedOk(VArr.Size()))
					{
						bLegConverged = true;
						break;
					}
				}
				const double U = DepVinf.Size();
				if (!bLegConverged || U < 1.0 || !SpeedOk(VArr.Size()))
				{
					continue;
				}

				FSearchNode Node;
				Node.MassTotal = Init.TotalMass();
				Node.BodyId = TargetId;
				Node.ArrivalTime = ArrivalTime;
				Node.VinInf = VArr - TVel;
				Node.TotalDeltaV = FMath::Sqrt(U * U + 2.0 * Parking.Body->Mu / Parking.Radius) - Parking.VCirc;
				const double Burn = TryBurn(Node, Node.TotalDeltaV);
				if (Burn < 0.0)
				{
					continue;
				}
				Node.FuelUsed = Burn;
				Node.MassTotal -= Burn;
				Node.Visits = { Parking.Body->Id, TargetId };

				FManeuverEvent& Maneuver = Node.Maneuvers.AddDefaulted_GetRef();
				Maneuver.Time = BurnTime;
				Maneuver.DeltaV = (DepVinf / U) * Node.TotalDeltaV;
				Maneuver.Note = FName(TEXT("departure"));

				FEdgeRecord& Edge = Node.Edges.AddDefaulted_GetRef();
				Edge.FromBody = Parking.Body->Id;
				Edge.ToBody = TargetId;
				Edge.DepartureTime = BurnTime;
				Edge.ArrivalTime = ArrivalTime;
				Edge.DepVinf = DepVinf;
				Edge.ArrVinf = Node.VinInf;

				Level0.Add(Node);
			}
		}
	}
	} // parking-orbit start

	// ------------------------------------------------------------------
	// Direct powered escape (no flybies) from the start state.
	// ------------------------------------------------------------------
	if (bHeliocentricStart)
	{
		for (int32 CoastIndex = 0; CoastIndex < Search.WaitSteps; ++CoastIndex)
		{
			const double DepartureTime = Init.Epoch
				+ Search.MaxDepartureWaitDays * DaySeconds * CoastIndex / FMath::Max(1, Search.WaitSteps - 1);
			FVector CoastPos;
			FVector CoastVel;
			OrbitMath::Propagate(Init.Position, Init.Velocity, Star->Mu, DepartureTime - Init.Epoch, CoastPos, CoastVel);
			const double SpeedNow = CoastVel.Size();
			if (SpeedNow < 1.0)
			{
				continue;
			}

			for (int32 AngleIndex = 0; AngleIndex < 5; ++AngleIndex)
			{
				const FVector Dir = RotateAroundZ(CoastVel / SpeedNow,
					FMath::DegreesToRadians(-90.0 + 45.0 * AngleIndex));
				for (int32 UIndex = 0; UIndex < 8; ++UIndex)
				{
					const double U = FMath::Exp(FMath::Lerp(
						FMath::Loge(FMath::Max(500.0, 0.3 * SpeedNow)), FMath::Loge(8.0e4), UIndex / 7.0));

					FSearchNode Node;
					Node.MassTotal = Init.TotalMass();
					Node.bHeliocentricStart = true;
					Node.FirstLegBurn.Time = DepartureTime;
					Node.FirstLegBurn.DeltaV = Dir * U - CoastVel;
					Node.FirstLegBurn.Note = FName(TEXT("escape burn"));
					Node.TotalDeltaV = Node.FirstLegBurn.DeltaV.Size();
					const double Burn = TryBurn(Node, Node.TotalDeltaV);
					if (Burn < 0.0)
					{
						continue;
					}

					const FVector VNew = Dir * U;
					if (!SpeedOk(VNew.Size()))
					{
						continue;
					}
					double CoastToBoundary;
					constexpr double ReplanEscapeMargin = 1.2;
					if (!TimeToReachDistance(CoastPos, VNew, Star->Mu, Boundary * ReplanEscapeMargin,
						MaxFlightTime - DepartureTime, CoastToBoundary))
					{
						continue;
					}

					FCandidate Candidate;
					Candidate.bHeliocentricStart = true;
					Candidate.FirstLegBurn = Node.FirstLegBurn;
					Candidate.TotalDeltaV = Node.TotalDeltaV;
					Candidate.FlightTime = DepartureTime + CoastToBoundary;
					Candidate.RiskScore = 0.0;
					Candidate.RouteBodies = { Star->Id };
					FManeuverEvent& Maneuver = Candidate.Maneuvers.AddDefaulted_GetRef();
					Maneuver = Node.FirstLegBurn;

					FEdgeRecord& Edge = Candidate.Edges.AddDefaulted_GetRef();
					Edge.FromBody = Star->Id;
					Edge.ToBody = NAME_None;
					Edge.DepartureTime = DepartureTime;
					Edge.ArrivalTime = Candidate.FlightTime;
					Edge.DepVinf = VNew;

					Candidate.Cost = EvaluateCost(Candidate.TotalDeltaV, Candidate.FlightTime, 0.0);
					Candidates.Add(Candidate);
				}
			}
		}
	}
	else
	{
		FVector APos;
		FVector AVel;
		System.GetBodyWorldState(Parking.Body->Id, 0.0, APos, AVel);
		const FVector ProgradeDir = AVel.GetSafeNormal();

		for (int32 AngleIndex = 0; AngleIndex < 5; ++AngleIndex)
		{
			const double OffsetDeg = -90.0 + 45.0 * AngleIndex;
			const FVector Dir = RotateAroundZ(ProgradeDir, FMath::DegreesToRadians(OffsetDeg));
			const double BurnTime = ProgradeBurnTime(Parking, Dir, 0.0);
			System.GetBodyWorldState(Parking.Body->Id, BurnTime, APos, AVel);

			for (int32 UIndex = 0; UIndex < 8; ++UIndex)
			{
				const double U = FMath::Exp(FMath::Lerp(FMath::Loge(1.0e3), FMath::Loge(6.0e4), UIndex / 7.0));
				const FVector VHeliocentric = AVel + Dir * U;
				if (!SpeedOk(VHeliocentric.Size()))
				{
					continue;
				}

				FSearchNode Node;
				Node.MassTotal = Init.TotalMass();
				Node.TotalDeltaV = FMath::Sqrt(U * U + 2.0 * Parking.Body->Mu / Parking.Radius) - Parking.VCirc;
				const double Burn = TryBurn(Node, Node.TotalDeltaV);
				if (Burn < 0.0)
				{
					continue;
				}

				double CoastTime;
				const FVector DeparturePos = Parking.WorldPositionAt(System, BurnTime);
				// Margin over the boundary: the SOI-exit patch and finite-burn
				// execution shave a little energy off marginal escapes.
				constexpr double DirectEscapeMargin = 1.2;
				if (!TimeToReachDistance(DeparturePos, VHeliocentric, Star->Mu, Boundary * DirectEscapeMargin,
					MaxFlightTime - BurnTime, CoastTime))
				{
					continue;
				}

				FCandidate Candidate;
				Candidate.TotalDeltaV = Node.TotalDeltaV;
				Candidate.FlightTime = BurnTime + CoastTime;
				Candidate.RiskScore = 0.0;
				Candidate.RouteBodies = { Parking.Body->Id };
				FManeuverEvent& Maneuver = Candidate.Maneuvers.AddDefaulted_GetRef();
				Maneuver.Time = BurnTime;
				Maneuver.DeltaV = Dir * Node.TotalDeltaV;
				Maneuver.Note = FName(TEXT("escape burn"));

				FEdgeRecord& Edge = Candidate.Edges.AddDefaulted_GetRef();
				Edge.FromBody = Parking.Body->Id;
				Edge.ToBody = NAME_None;
				Edge.DepartureTime = BurnTime;
				Edge.ArrivalTime = Candidate.FlightTime;
				Edge.DepVinf = Dir * U;

				Candidate.Cost = EvaluateCost(Candidate.TotalDeltaV, Candidate.FlightTime, 0.0);
				Candidates.Add(Candidate);
			}
		}
	} // parking-orbit direct escape

	// ------------------------------------------------------------------
	// Beam levels: flyby legs between planets, escape edges to the boundary.
	// ------------------------------------------------------------------
	Level0.Sort([&](const FSearchNode& A, const FSearchNode& B)
	{
		return EvaluateCost(A.TotalDeltaV, A.ArrivalTime, A.MaxRisk) < EvaluateCost(B.TotalDeltaV, B.ArrivalTime, B.MaxRisk);
	});
	Beam = TArray<FSearchNode>(Level0.GetData(), FMath::Min(Level0.Num(), Search.BeamWidth));

	for (int32 Level = 0; Level < Search.MaxHops; ++Level)
	{
		// Escape candidates from every current beam node (unpowered assist turns).
		for (const FSearchNode& Node : Beam)
		{
			const FCelestialBodyDesc* B = System.FindBody(Node.BodyId);
			const double UIn = Node.VinInf.Size();
			const double Transit = 2.0 * B->SoiRadius / FMath::Max(UIn, 1.0);
			const double DepartureTime = Node.ArrivalTime + Transit;
			if (DepartureTime > MaxFlightTime)
			{
				continue;
			}

			FVector BPos;
			FVector BVel;
			System.GetBodyWorldState(B->Id, DepartureTime, BPos, BVel);

			const double SafeRp = B->Radius * Search.PeriapsisSafetyFactor;
			const double DeltaMax = MaxTurnAngle(B->Mu, SafeRp, UIn);

			for (int32 TurnIndex = 0; TurnIndex < 5; ++TurnIndex)
			{
				const double Turn = (TurnIndex / 4.0 * 2.0 - 1.0) * 0.95 * DeltaMax;
				const FVector VOut = RotateAroundZ(Node.VinInf, Turn);
				const FVector VHeliocentric = BVel + VOut;
				if (!SpeedOk(VHeliocentric.Size()))
				{
					continue;
				}

				double Rp = PeriapsisForTurn(FMath::Abs(Turn), B->Mu, UIn);
				Rp = FMath::Clamp(Rp, SafeRp, 0.8 * B->SoiRadius);
				const double Risk = RiskModel::EvaluateFlybyRisk(
					Request.RiskParams, Rp, B->Radius, UIn, Node.FuelUsed / Init.FuelMass);
				const double MaxRisk = FMath::Max(Node.MaxRisk, Risk);

				// Flyby escape legs carry an apoapsis margin: the ideal-turn model
				// overestimates what execution delivers, and a marginally-reaching
				// conic turns into a time-limit failure in flight.
				constexpr double EscapeMarginFactor = 1.2;
				double CoastTime;
				if (!TimeToReachDistance(BPos, VHeliocentric, Star->Mu, Boundary * EscapeMarginFactor,
					MaxFlightTime - DepartureTime, CoastTime))
				{
					continue;
				}

				FCandidate Candidate;
				Candidate.TotalDeltaV = Node.TotalDeltaV;
				Candidate.FlightTime = DepartureTime + CoastTime;
				Candidate.RiskScore = MaxRisk;
				Candidate.Maneuvers = Node.Maneuvers;
				Candidate.RouteBodies = Node.Visits;
				Candidate.Edges = Node.Edges;
				Candidate.bHeliocentricStart = Node.bHeliocentricStart;
				Candidate.FirstLegBurn = Node.FirstLegBurn;

				FEdgeRecord& Edge = Candidate.Edges.AddDefaulted_GetRef();
				Edge.FromBody = Node.BodyId;
				Edge.ToBody = NAME_None;
				Edge.DepartureTime = DepartureTime;
				Edge.ArrivalTime = Candidate.FlightTime;
				Edge.DepVinf = VOut;
				Edge.PeriapsisRadius = Rp;

				Candidate.Cost = EvaluateCost(Candidate.TotalDeltaV, Candidate.FlightTime, MaxRisk);
				Candidates.Add(Candidate);
			}
		}

		if (Level == Search.MaxHops - 1 || Beam.IsEmpty())
		{
			break;
		}

		// Expand: one more Lambert leg with a powered assist at the current body.
		TArray<FSearchNode> Next;
		for (const FSearchNode& Node : Beam)
		{
			const FCelestialBodyDesc* B = System.FindBody(Node.BodyId);
			const double UIn = Node.VinInf.Size();
			const double Transit = 2.0 * B->SoiRadius / FMath::Max(UIn, 1.0);
			const double DepartureTime = Node.ArrivalTime + Transit;
			const double SafeRp = B->Radius * Search.PeriapsisSafetyFactor;

			for (const FName& TargetId : PlanetIds)
			{
				if (TargetId == Node.BodyId)
				{
					continue;
				}
				for (int32 TofIndex = 0; TofIndex < Search.FlightTimeSteps; ++TofIndex)
				{
					const double Tof = LogTof(TofIndex);
					const double ArrivalTime = DepartureTime + Tof;
					if (ArrivalTime > MaxFlightTime)
					{
						continue;
					}

					const FCelestialBodyDesc* Target = System.FindBody(TargetId);
					FVector BPos;
					FVector BVel;
					FVector TPos;
					FVector TVel;
					System.GetBodyWorldState(B->Id, DepartureTime, BPos, BVel);
					System.GetBodyWorldState(TargetId, ArrivalTime, TPos, TVel);

					FVector VDep;
					FVector VArr;
					if (!SolveLambertPrograde(BPos, TPos, Star->Mu, Tof, VDep, VArr))
					{
						continue;
					}
					if (!SpeedOk(VArr.Size()))
					{
						continue;
					}

					const FVector DepVinf = VDep - BVel;
					const double UOut = DepVinf.Size();
					if (UOut < 1.0)
					{
						continue;
					}

					// Turn feasibility at the safe periapsis (hard rule — inadmissible, not expensive).
					const double TurnNeeded = FMath::Acos(FMath::Clamp(
						FVector::DotProduct(Node.VinInf / UIn, DepVinf / UOut), -1.0, 1.0));
					const double DeltaMax = MaxTurnAngle(B->Mu, SafeRp, UOut);
					if (TurnNeeded > DeltaMax * 1.02)
					{
						continue;
					}

					// Powered assist: magnitude patch at periapsis.
					const double Patch = FMath::Abs(UOut - UIn);
					const double Burn = TryBurn(Node, Patch);
					if (Burn < 0.0)
					{
						continue;
					}

					double Rp = PeriapsisForTurn(TurnNeeded, B->Mu, UOut);
					Rp = FMath::Clamp(Rp, SafeRp, 0.8 * B->SoiRadius);
					const double Risk = RiskModel::EvaluateFlybyRisk(
						Request.RiskParams, Rp, B->Radius, UOut, (Node.FuelUsed + Burn) / Init.FuelMass);

					FSearchNode NewNode = Node;
					NewNode.BodyId = TargetId;
					NewNode.ArrivalTime = ArrivalTime;
					NewNode.VinInf = VArr - TVel;
					NewNode.TotalDeltaV += Patch;
					NewNode.FuelUsed += Burn;
					NewNode.MassTotal -= Burn;
					NewNode.MaxRisk = FMath::Max(NewNode.MaxRisk, Risk);
					NewNode.Visits.Add(TargetId);

					if (Patch > 0.05)
					{
						// The burn happens at periapsis, along the velocity bisector of the flyby.
						const FVector Bisector = (Node.VinInf / UIn + DepVinf / UOut).GetSafeNormal();
						const double VPeriIn = FMath::Sqrt(UIn * UIn + 2.0 * B->Mu / Rp);
						const double VPeriOut = FMath::Sqrt(UOut * UOut + 2.0 * B->Mu / Rp);
						FManeuverEvent& Maneuver = NewNode.Maneuvers.AddDefaulted_GetRef();
						Maneuver.Time = Node.ArrivalTime + 0.5 * Transit;
						Maneuver.DeltaV = Bisector * (VPeriOut - VPeriIn);
						Maneuver.Note = FName(TEXT("assist patch"));
					}

					FEdgeRecord& Edge = NewNode.Edges.AddDefaulted_GetRef();
					Edge.FromBody = Node.BodyId;
					Edge.ToBody = TargetId;
					Edge.DepartureTime = DepartureTime;
					Edge.ArrivalTime = ArrivalTime;
					Edge.DepVinf = DepVinf;
					Edge.ArrVinf = NewNode.VinInf;
					Edge.PeriapsisRadius = Rp;

					Next.Add(NewNode);
				}
			}
		}

		Next.Sort([&](const FSearchNode& A, const FSearchNode& B)
		{
			return EvaluateCost(A.TotalDeltaV, A.ArrivalTime, A.MaxRisk) < EvaluateCost(B.TotalDeltaV, B.ArrivalTime, B.MaxRisk);
		});
		Beam = TArray<FSearchNode>(Next.GetData(), FMath::Min(Next.Num(), Search.BeamWidth));
	}

	if (Candidates.IsEmpty())
	{
		Result.FailReason = TEXT("No feasible route found under the configured limits");
		return Result;
	}

	Candidates.Sort(CandidateOrder);

	// ------------------------------------------------------------------
	// Refinement: B-plane aiming on intermediate flybies, so execution
	// actually hits the intended periapsis instead of the planet center
	// (a center-aimed pass is a collision course).
	// ------------------------------------------------------------------
	auto RefineCandidate = [&System, &Parking, &Star, &Search, &bHeliocentricStart, &Init](FCandidate& Best)
	{
		for (int32 Iteration = 0; Iteration < 2 && Best.Edges.Num() > 1; ++Iteration)
		{
			for (int32 EdgeIndex = 1; EdgeIndex < Best.Edges.Num(); ++EdgeIndex)
			{
				FEdgeRecord& Incoming = Best.Edges[EdgeIndex - 1];
				FEdgeRecord& Outgoing = Best.Edges[EdgeIndex];
				if (Incoming.ToBody == NAME_None)
				{
					continue;
				}
				const FCelestialBodyDesc* B = System.FindBody(Incoming.ToBody);

				const double UIn = Incoming.ArrVinf.Size();
				const double UOut = Outgoing.DepVinf.Size();
				if (UIn < 1.0 || UOut < 1.0)
				{
					continue;
				}
				const double TurnNeeded = FMath::Acos(FMath::Clamp(
					FVector::DotProduct(Incoming.ArrVinf / UIn, Outgoing.DepVinf / UOut), -1.0, 1.0));
				const double SafeRp = B->Radius * Search.PeriapsisSafetyFactor;
				if (TurnNeeded > MaxTurnAngle(B->Mu, SafeRp, UOut) * 1.02)
				{
					continue;
				}

				double Rp = FMath::Clamp(PeriapsisForTurn(TurnNeeded, B->Mu, UOut), SafeRp, 0.8 * B->SoiRadius);
				const double Ecc = 1.0 + Rp * UOut * UOut / B->Mu;
				const double Bmag = FMath::Sqrt(B->Mu * Rp * (1.0 + Ecc)) / UOut;

				FVector Bin = FVector::CrossProduct(FVector::UpVector, Incoming.ArrVinf);
				if (Bin.Size() < 1.0)
				{
					continue;
				}
				const double TurnSign = FVector::DotProduct(
					FVector::CrossProduct(Incoming.ArrVinf, Outgoing.DepVinf), FVector::UpVector) >= 0.0 ? 1.0 : -1.0;

				FVector BPos;
				FVector BVel;
				System.GetBodyWorldState(B->Id, Incoming.ArrivalTime, BPos, BVel);
				const FVector AimPoint = BPos + TurnSign * Bmag * Bin.GetSafeNormal();

				// Re-solve the incoming leg to the offset aim point (same times).
				FVector DeparturePos;
				if (EdgeIndex == 1)
				{
					if (bHeliocentricStart)
					{
						FVector IgnoredVel;
						OrbitMath::Propagate(Init.Position, Init.Velocity, Star->Mu,
							Incoming.DepartureTime - Init.Epoch, DeparturePos, IgnoredVel);
					}
					else
					{
						DeparturePos = Parking.WorldPositionAt(System, Best.Edges[0].DepartureTime);
					}
				}
				else
				{
					const FCelestialBodyDesc* PrevBody = System.FindBody(Best.Edges[EdgeIndex - 1].FromBody);
					FVector PrevPos;
					FVector PrevVel;
					System.GetBodyWorldState(PrevBody->Id, Incoming.DepartureTime, PrevPos, PrevVel);
					DeparturePos = PrevPos;
				}

				FVector VDep;
				FVector VArr;
				if (SolveLambertPrograde(DeparturePos, AimPoint, Star->Mu,
					Incoming.ArrivalTime - Incoming.DepartureTime, VDep, VArr))
				{
					const FName DepartureBodyId = Best.Edges[EdgeIndex - 1].FromBody;
					FVector DepartureBodyPos;
					FVector DepartureBodyVel;
					System.GetBodyWorldState(DepartureBodyId, Incoming.DepartureTime, DepartureBodyPos, DepartureBodyVel);
					Incoming.DepVinf = VDep - DepartureBodyVel;
					Incoming.ArrVinf = VArr - BVel;
					Incoming.PeriapsisRadius = Rp;
				}
			}
		}
	};

	// ------------------------------------------------------------------
	// Build the executable plan from (refined) edges.
	// ------------------------------------------------------------------
	auto BuildPlan = [&System, &Init, &Parking, &Search, &Request](const FCandidate& In, FMissionPlan& OutPlan, double& OutTotalDeltaV, double& OutRisk) -> bool
	{
		OutPlan.Maneuvers.Reset();
		OutTotalDeltaV = 0.0;
		OutRisk = 0.0;
		double MassTotal = Init.TotalMass();
		double FuelUsed = 0.0;

		auto TrySpend = [&](double DeltaV) -> bool
		{
			const double Burn = OrbitMath::FuelBurned(MassTotal, DeltaV, Init.Isp);
			if (FuelUsed + Burn > Init.FuelMass)
			{
				return false;
			}
			FuelUsed += Burn;
			MassTotal -= Burn;
			return true;
		};

		// Departure burn.
		if (In.Edges.IsEmpty())
		{
			return false;
		}
		const FEdgeRecord& First = In.Edges[0];
		if (In.bHeliocentricStart)
		{
			// Replan start: the corrective burn off the current conic is stored verbatim.
			const double FirstDv = In.FirstLegBurn.DeltaV.Size();
			if (!TrySpend(FirstDv))
			{
				return false;
			}
			OutTotalDeltaV += FirstDv;
			FManeuverEvent& Maneuver = OutPlan.Maneuvers.AddDefaulted_GetRef();
			Maneuver = In.FirstLegBurn;
		}
		else if (First.ToBody != NAME_None)
		{
			const double U = First.DepVinf.Size();
			const double DepartureDeltaV = FMath::Sqrt(U * U + 2.0 * Parking.Body->Mu / Parking.Radius) - Parking.VCirc;
			if (!TrySpend(DepartureDeltaV))
			{
				return false;
			}
			OutTotalDeltaV += DepartureDeltaV;
			FManeuverEvent& Maneuver = OutPlan.Maneuvers.AddDefaulted_GetRef();
			Maneuver.Time = First.DepartureTime;
			Maneuver.DeltaV = (First.DepVinf / U) * DepartureDeltaV;
			Maneuver.Note = FName(TEXT("departure"));
		}
		else if (First.ToBody == NAME_None && In.Edges.Num() == 1)
		{
			// Direct escape stored its burn already; rebuild from DepVinf.
			const double U = First.DepVinf.Size();
			const double DepartureDeltaV = FMath::Sqrt(U * U + 2.0 * Parking.Body->Mu / Parking.Radius) - Parking.VCirc;
			if (!TrySpend(DepartureDeltaV))
			{
				return false;
			}
			OutTotalDeltaV += DepartureDeltaV;
			FManeuverEvent& Maneuver = OutPlan.Maneuvers.AddDefaulted_GetRef();
			Maneuver.Time = First.DepartureTime;
			Maneuver.DeltaV = (First.DepVinf / U) * DepartureDeltaV;
			Maneuver.Note = FName(TEXT("escape burn"));
		}

		// Intermediate flyby patches.
		for (int32 EdgeIndex = 1; EdgeIndex < In.Edges.Num(); ++EdgeIndex)
		{
			const FEdgeRecord& Incoming = In.Edges[EdgeIndex - 1];
			const FEdgeRecord& Outgoing = In.Edges[EdgeIndex];
			if (Incoming.ToBody == NAME_None)
			{
				continue;
			}
			const FCelestialBodyDesc* B = System.FindBody(Incoming.ToBody);
			const double UIn = Incoming.ArrVinf.Size();
			const double UOut = Outgoing.DepVinf.Size();
			if (UIn < 1.0 || UOut < 1.0)
			{
				continue;
			}

			double Rp = Incoming.PeriapsisRadius;
			if (Rp <= 0.0)
			{
				Rp = B->Radius * Search.PeriapsisSafetyFactor;
			}
			const double VPeriIn = FMath::Sqrt(UIn * UIn + 2.0 * B->Mu / Rp);
			const double VPeriOut = FMath::Sqrt(UOut * UOut + 2.0 * B->Mu / Rp);
			const double Patch = FMath::Abs(VPeriOut - VPeriIn);
			if (Patch > 0.05)
			{
				if (!TrySpend(Patch))
				{
					return false;
				}
				OutTotalDeltaV += Patch;
				const FVector Bisector = (Incoming.ArrVinf / UIn + Outgoing.DepVinf / UOut).GetSafeNormal();
				FManeuverEvent& Maneuver = OutPlan.Maneuvers.AddDefaulted_GetRef();
				Maneuver.Time = Incoming.ArrivalTime + 0.5 * (Outgoing.DepartureTime - Incoming.ArrivalTime);
				Maneuver.DeltaV = Bisector * (VPeriOut - VPeriIn);
				Maneuver.Note = FName(TEXT("assist patch"));
			}

			const double Risk = RiskModel::EvaluateFlybyRisk(
				Request.RiskParams, Rp, B->Radius, UOut, FuelUsed / Init.FuelMass);
			OutRisk = FMath::Max(OutRisk, Risk);
		}

		OutPlan.Maneuvers.Sort([](const FManeuverEvent& A, const FManeuverEvent& B) { return A.Time < B.Time; });
		OutPlan.TotalDeltaV = OutTotalDeltaV;
		OutPlan.FlightTime = In.FlightTime;
		OutPlan.RiskScore = OutRisk;
		return true;
	};

	// ------------------------------------------------------------------
	// Plan validation: fly the best candidates through the exact patched-conics
	// model execution uses, and emit the first one that reaches the boundary.
	// The search model approximates flybies (instant turn, periapsis timing),
	// so a route is only "a plan" once it has actually flown.
	// ------------------------------------------------------------------
	constexpr int32 MaxValidationAttempts = 16;
	for (int32 Attempt = 0; Attempt < Candidates.Num() && Attempt < MaxValidationAttempts; ++Attempt)
	{
		FCandidate Best = Candidates[Attempt];
		RefineCandidate(Best);

		FMissionPlan BuiltPlan;
		double BuiltDeltaV = 0.0;
		double BuiltRisk = 0.0;
		if (!BuildPlan(Best, BuiltPlan, BuiltDeltaV, BuiltRisk))
		{
			continue;
		}

		const FMissionRunResult Playback = MissionRunner::Run(System, Init, BuiltPlan, Request.Limits);
		if (!Playback.bSuccess)
		{
			continue;
		}

		Result.bSuccess = true;
		Result.Plan = MoveTemp(BuiltPlan);
		Result.TotalDeltaV = BuiltDeltaV;
		Result.RiskScore = BuiltRisk;
		Result.FlightTime = Playback.FinishTime;
		Result.Cost = Best.Cost;
		Result.RouteBodies = Best.RouteBodies;
		return Result;
	}

	Result.FailReason = FString::Printf(TEXT("None of the %d best routes survived playback validation"),
		FMath::Min(Candidates.Num(), MaxValidationAttempts));
	return Result;
}
} // namespace RoutePlanner
