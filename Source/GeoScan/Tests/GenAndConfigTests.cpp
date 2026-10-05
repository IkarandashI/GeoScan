#if WITH_DEV_AUTOMATION_TESTS

#include "Misc/AutomationTest.h"
#include "Config/ScenarioConfig.h"
#include "Config/ScenarioJson.h"
#include "Core/OrbitMath.h"
#include "Gen/SystemGenerator.h"

#if WITH_EDITOR
#include "Actors/CelestialBodyActor.h"
#include "Actors/SpacecraftActor.h"
#include "AssetRegistry/AssetRegistryModule.h"
#include "HAL/FileManager.h"
#include "Misc/PackageName.h"
#include "UObject/Package.h"
#include "UObject/SavePackage.h"
#endif

namespace
{
	bool FVectorsNearlyEqual(const FVector& A, const FVector& B, double RelativeTolerance)
	{
		const double Scale = FMath::Max(1.0, FMath::Max(A.Size(), B.Size()));
		return (A - B).Size() <= RelativeTolerance * Scale;
	}
	FSystemGenParams MakeTestParams(uint64 Seed = 12345)
	{
		FSystemGenParams Params;
		Params.Seed = Seed;
		Params.NumPlanets = 6;
		return Params;
	}

	bool SystemsIdentical(const FPlanetarySystem& A, const FPlanetarySystem& B)
	{
		if (A.Bodies.Num() != B.Bodies.Num() || A.SystemBoundary != B.SystemBoundary)
		{
			return false;
		}
		for (int32 i = 0; i < A.Bodies.Num(); ++i)
		{
			const FCelestialBodyDesc& BodyA = A.Bodies[i];
			const FCelestialBodyDesc& BodyB = B.Bodies[i];
			// Exact comparison on purpose: generation must be bit-identical for a seed.
			if (BodyA.Id != BodyB.Id || BodyA.ParentId != BodyB.ParentId
				|| BodyA.Mass != BodyB.Mass || BodyA.Radius != BodyB.Radius
				|| BodyA.OrbitRadius != BodyB.OrbitRadius || BodyA.InitialPhase != BodyB.InitialPhase
				|| BodyA.Inclination != BodyB.Inclination || BodyA.SoiRadius != BodyB.SoiRadius
				|| BodyA.OrbitalPeriod != BodyB.OrbitalPeriod || BodyA.OrbitalSpeed != BodyB.OrbitalSpeed)
			{
				return false;
			}
		}
		return true;
	}
}

// ---------------------------------------------------------------- Generation

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGenDeterminismTest, "GeoScan.Gen.Determinism",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FGenDeterminismTest::RunTest(const FString&)
{
	const FPlanetarySystem First = SystemGen::Generate(MakeTestParams(987));
	const FPlanetarySystem Second = SystemGen::Generate(MakeTestParams(987));
	TestTrue("Same seed -> identical system", SystemsIdentical(First, Second));

	const FPlanetarySystem Third = SystemGen::Generate(MakeTestParams(988));
	TestFalse("Different seed -> different system", SystemsIdentical(First, Third));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGenConstraintsTest, "GeoScan.Gen.Constraints",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FGenConstraintsTest::RunTest(const FString&)
{
	const FSystemGenParams Params = MakeTestParams();
	const FPlanetarySystem System = SystemGen::Generate(Params);

	TestEqual("Star + N planets", System.Bodies.Num(), Params.NumPlanets + 1);
	TestEqual("Star is Bodies[0]", System.GetStar().Id.ToString(), FString(TEXT("Star")));

	double PreviousOrbit = 0.0;
	bool bOrbitsOk = true;
	for (int32 i = 1; i < System.Bodies.Num(); ++i)
	{
		const FCelestialBodyDesc& Planet = System.Bodies[i];
		// 1 m slack: exp(lerp(log min, log max)) can miss the exact bounds by an ulp.
		bOrbitsOk &= Planet.OrbitRadius >= Params.MinOrbitalRadius - 1.0
			&& Planet.OrbitRadius <= Params.MaxOrbitalRadius + 1.0;
		bOrbitsOk &= Planet.OrbitRadius > PreviousOrbit;
		bOrbitsOk &= Planet.Radius >= Params.PlanetRadiusMin && Planet.Radius <= Params.PlanetRadiusMax;
		bOrbitsOk &= Planet.Mass >= Params.PlanetMassMin && Planet.Mass <= Params.PlanetMassMax;
		bOrbitsOk &= Planet.SoiRadius > 0.0 && Planet.ParentId == System.GetStar().Id;
		PreviousOrbit = Planet.OrbitRadius;
	}
	TestTrue("Orbits monotonic and inside [min, max]", bOrbitsOk);

	TestTrue("Boundary: fixed mode", FMath::IsNearlyEqual(System.SystemBoundary, Params.FixedBoundaryDistance, 1.0));

	FSystemGenParams RelativeParams = MakeTestParams();
	RelativeParams.BoundaryMode = ESystemBoundaryMode::RelativeToOutermost;
	RelativeParams.BoundaryK = 2.0;
	const FPlanetarySystem RelativeSystem = SystemGen::Generate(RelativeParams);
	TestTrue("Boundary: K * outermost orbit",
		FMath::IsNearlyEqual(RelativeSystem.SystemBoundary, 2.0 * RelativeSystem.OutermostChildOrbitRadius(), 1.0));

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FGenEphemerisTest, "GeoScan.Gen.Ephemeris",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FGenEphemerisTest::RunTest(const FString&)
{
	const FPlanetarySystem System = SystemGen::Generate(MakeTestParams());
	const FCelestialBodyDesc& Star = System.GetStar();

	for (int32 i = 1; i < System.Bodies.Num(); ++i)
	{
		const FCelestialBodyDesc& Planet = System.Bodies[i];

		FVector Position;
		FVector Velocity;
		TestTrue(FString::Printf(TEXT("World state resolvable: %s"), *Planet.Id.ToString()),
			System.GetBodyWorldState(Planet.Id, 1000.0, Position, Velocity));

		// Circular orbit around a static star: speed and radius match the closed form.
		TestTrue(FString::Printf(TEXT("Speed matches circular orbit: %s"), *Planet.Id.ToString()),
			FMath::IsNearlyEqual(Velocity.Size(), OrbitMath::CircularOrbitSpeed(Star.Mu, Planet.OrbitRadius), 1e-3));
		TestTrue(FString::Printf(TEXT("Radius matches orbit: %s"), *Planet.Id.ToString()),
			FMath::IsNearlyEqual(Position.Size(), Planet.OrbitRadius, 1e-3));

		// After a full period the body returns to the state it had at t=0.
		FVector PositionAtStart;
		FVector IgnoredStartVelocity;
		FVector PositionAfterPeriod;
		FVector IgnoredVelocity;
		System.GetBodyWorldState(Planet.Id, 0.0, PositionAtStart, IgnoredStartVelocity);
		System.GetBodyWorldState(Planet.Id, Planet.OrbitalPeriod, PositionAfterPeriod, IgnoredVelocity);
		TestTrue(FString::Printf(TEXT("Period round trip: %s"), *Planet.Id.ToString()),
			FVectorsNearlyEqual(PositionAfterPeriod, PositionAtStart, 1e-9));
	}

	return true;
}

// ---------------------------------------------------------------- Config

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FConfigModeWeightsTest, "GeoScan.Config.ModeWeights",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FConfigModeWeightsTest::RunTest(const FString&)
{
	UScenarioConfig* Config = NewObject<UScenarioConfig>(GetTransientPackage());

	auto WeightOf = [&Config](FName Id) -> double
	{
		for (const FCriterionWeight& Criterion : Config->Optimization.Criteria)
		{
			if (Criterion.CriterionId == Id)
			{
				return Criterion.Weight;
			}
		}
		return -1.0;
	};

	Config->Optimization.Mode = EOptimizationMode::FuelEfficient;
	Config->SyncWeightsToMode();
	TestTrue("Fuel mode: Fuel=1, Time=0, Risk=0",
		WeightOf("Fuel") == 1.0 && WeightOf("Time") == 0.0 && WeightOf("Risk") == 0.0);

	Config->Optimization.Mode = EOptimizationMode::TimeEfficient;
	Config->SyncWeightsToMode();
	TestTrue("Time mode: Fuel=0, Time=1, Risk=0",
		WeightOf("Fuel") == 0.0 && WeightOf("Time") == 1.0 && WeightOf("Risk") == 0.0);

	Config->Optimization.Mode = EOptimizationMode::Balanced;
	Config->SyncWeightsToMode();
	TestTrue("Balanced mode: 0.6/0.3/0.1",
		WeightOf("Fuel") == 0.6 && WeightOf("Time") == 0.3 && WeightOf("Risk") == 0.1);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FConfigValidationTest, "GeoScan.Config.Validation",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FConfigValidationTest::RunTest(const FString&)
{
	UScenarioConfig* Config = NewObject<UScenarioConfig>(GetTransientPackage());
	Config->ApplyPreset(EOptimizationMode::Balanced);

	const FPlanetarySystem System = SystemGen::Generate(Config->ToGenParams());
	TestTrue("Preset passes validation against its system",
		Config->Validate(&System).IsEmpty());

	// Starting body must not be the outermost planet (task requirement).
	FName OutermostBody;
	double OutermostOrbit = -1.0;
	for (const FCelestialBodyDesc& Body : System.Bodies)
	{
		if (Body.ParentId != NAME_None && Body.OrbitRadius > OutermostOrbit)
		{
			OutermostOrbit = Body.OrbitRadius;
			OutermostBody = Body.Id;
		}
	}
	Config->Spacecraft.StartingBodyId = OutermostBody;
	TArray<FString> Errors = Config->Validate(&System);
	bool bOutermostCaught = false;
	for (const FString& Error : Errors)
	{
		bOutermostCaught |= Error.Contains(TEXT("outermost"));
	}
	TestTrue("Outermost starting body rejected", bOutermostCaught);

	Config->Spacecraft.StartingBodyId = FName(TEXT("Planet_1"));
	Config->Spacecraft.StartingOrbitRadius = 1.0; // below the surface
	Errors = Config->Validate(&System);
	bool bSurfaceCaught = false;
	for (const FString& Error : Errors)
	{
		bSurfaceCaught |= Error.Contains(TEXT("surface"));
	}
	TestTrue("Sub-surface starting orbit rejected", bSurfaceCaught);

	Config->Spacecraft.StartingOrbitRadius = 1.0e30; // beyond any SOI
	Errors = Config->Validate(&System);
	bool bSoiCaught = false;
	for (const FString& Error : Errors)
	{
		bSoiCaught |= Error.Contains(TEXT("sphere of influence"));
	}
	TestTrue("Outside-SOI starting orbit rejected", bSoiCaught);

	return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FConfigJsonRoundTripTest, "GeoScan.Config.JsonRoundTrip",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FConfigJsonRoundTripTest::RunTest(const FString&)
{
	UScenarioConfig* Original = NewObject<UScenarioConfig>(GetTransientPackage());
	Original->ApplyPreset(EOptimizationMode::FuelEfficient);
	Original->System.Seed = 9876543210LL;
	Original->System.NumPlanets = 9;
	Original->Spacecraft.DryMass = 1234.5;
	Original->Spacecraft.FuelMass = 4321.0;

	const FString FilePath = FPaths::Combine(FPaths::ProjectSavedDir(), TEXT("Tests"), TEXT("ScenarioRoundTrip.json"));
	TestTrue("Exported", ScenarioJson::ExportToFile(Original, FilePath));

	UScenarioConfig* Loaded = ScenarioJson::ImportFromFile(FilePath, GetTransientPackage());
	TestTrue("Imported", Loaded != nullptr);
	if (Loaded == nullptr)
	{
		return false;
	}

	TestEqual("Seed survived int64 round trip", Loaded->System.Seed, static_cast<int64>(9876543210LL));
	TestEqual("NumPlanets", Loaded->System.NumPlanets, 9);
	TestTrue("DryMass", FMath::IsNearlyEqual(Loaded->Spacecraft.DryMass, 1234.5, 1e-6));
	TestTrue("FuelMass", FMath::IsNearlyEqual(Loaded->Spacecraft.FuelMass, 4321.0, 1e-6));
	TestEqual("Mode", static_cast<int32>(Loaded->Optimization.Mode), static_cast<int32>(EOptimizationMode::FuelEfficient));
	TestEqual("Criteria count", Loaded->Optimization.Criteria.Num(), 3);

	return true;
}

#if WITH_EDITOR
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FConfigPresetAssetsTest, "GeoScan.Editor.CreatePresetAssets",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FConfigPresetAssetsTest::RunTest(const FString&)
{
	const TArray<TPair<EOptimizationMode, FString>> Presets = {
		{ EOptimizationMode::FuelEfficient, TEXT("DA_Scenario_FuelEfficient") },
		{ EOptimizationMode::TimeEfficient, TEXT("DA_Scenario_TimeEfficient") },
		{ EOptimizationMode::Balanced, TEXT("DA_Scenario_Balanced") },
	};

	for (const TPair<EOptimizationMode, FString>& Preset : Presets)
	{
		const FString PackagePath = FPaths::Combine(TEXT("/Game/Scenarios"), Preset.Value);
		const FString FileName = FPackageName::LongPackageNameToFilename(PackagePath, FPackageName::GetAssetPackageExtension());

		// Idempotent: drop a previously generated preset file before recreating it,
		// otherwise NewObject collides with the asset on disk and crashes the editor.
		IFileManager::Get().Delete(*FileName, false, true);

		UPackage* Package = CreatePackage(*PackagePath);
		if (!TestTrue(FString::Printf(TEXT("Package created: %s"), *Preset.Value), Package != nullptr))
		{
			continue;
		}

		UScenarioConfig* Config = NewObject<UScenarioConfig>(Package, *Preset.Value, RF_Public | RF_Standalone);
		Config->ApplyPreset(Preset.Key);

		// Wire authored Blueprint visuals when they exist, so freshly generated
		// presets use them (and test runs never strip visual setup from presets).
		if (UClass* StarBP = StaticLoadClass(ACelestialBodyActor::StaticClass(), nullptr, TEXT("/Game/Blueprints/BP_Star.BP_Star_C")))
		{
			Config->StarClass = StarBP;
		}
		if (UClass* PlanetBP = StaticLoadClass(ACelestialBodyActor::StaticClass(), nullptr, TEXT("/Game/Blueprints/BP_Planet.BP_Planet_C")))
		{
			Config->PlanetClass = PlanetBP;
		}
		if (UClass* SpacecraftBP = StaticLoadClass(ASpacecraftActor::StaticClass(), nullptr, TEXT("/Game/Blueprints/BP_Spacecraft.BP_Spacecraft_C")))
		{
			Config->SpacecraftClass = SpacecraftBP;
		}

		FAssetRegistryModule::AssetCreated(Config);
		Config->MarkPackageDirty();

		FSavePackageArgs SaveArgs;
		SaveArgs.TopLevelFlags = RF_Public | RF_Standalone;
		TestTrue(FString::Printf(TEXT("Saved: %s"), *Preset.Value),
			UPackage::SavePackage(Package, Config, *FileName, SaveArgs));
	}
	return true;
}
#endif


#if WITH_EDITOR
IMPLEMENT_SIMPLE_AUTOMATION_TEST(FPresetVisualClassesTest, "GeoScan.Editor.PresetVisualClasses",
	EAutomationTestFlags_ApplicationContextMask | EAutomationTestFlags::EngineFilter)

bool FPresetVisualClassesTest::RunTest(const FString&)
{
	for (const FString& PresetName : { TEXT("DA_Scenario_FuelEfficient"), TEXT("DA_Scenario_TimeEfficient"), TEXT("DA_Scenario_Balanced") })
	{
		const FString AssetPath = FString::Printf(TEXT("/Game/Scenarios/%s.%s"), *PresetName, *PresetName);
		UScenarioConfig* Preset = LoadObject<UScenarioConfig>(nullptr, *AssetPath);
		if (!TestTrue(FString::Printf(TEXT("%s loads"), *PresetName), Preset != nullptr))
		{
			continue;
		}

		auto CheckBodyClass = [&](const TCHAR* FieldName, const TSoftClassPtr<ACelestialBodyActor>& SoftClass)
		{
			UE_LOG(LogTemp, Warning, TEXT("[VisualDiag] %s.%s raw path = '%s'"),
				*PresetName, FieldName, *SoftClass.ToSoftObjectPath().ToString());
			if (!TestTrue(FString::Printf(TEXT("%s.%s wired"), *PresetName, FieldName), !SoftClass.IsNull()))
			{
				return;
			}
			UClass* Loaded = SoftClass.LoadSynchronous();
			if (!TestTrue(FString::Printf(TEXT("%s.%s resolves"), *PresetName, FieldName), Loaded != nullptr))
			{
				return;
			}
			ACelestialBodyActor* CDO = Loaded->GetDefaultObject<ACelestialBodyActor>();
			UE_LOG(LogTemp, Warning,
				TEXT("[VisualDiag] %s %s = %s | mesh = %s | material = %s"),
				*PresetName, FieldName, *Loaded->GetName(),
				CDO->GetBodyMesh()->GetStaticMesh() ? *CDO->GetBodyMesh()->GetStaticMesh()->GetName() : TEXT("NONE"),
				CDO->GetBodyMesh()->GetMaterial(0) ? *CDO->GetBodyMesh()->GetMaterial(0)->GetName() : TEXT("NONE"));
			TestTrue(FString::Printf(TEXT("%s.%s CDO has mesh"), *PresetName, FieldName),
				CDO->GetBodyMesh()->GetStaticMesh() != nullptr);
		};
		CheckBodyClass(TEXT("StarClass"), Preset->StarClass);
		CheckBodyClass(TEXT("PlanetClass"), Preset->PlanetClass);

		TestTrue(FString::Printf(TEXT("%s.SpacecraftClass wired"), *PresetName), !Preset->SpacecraftClass.IsNull());
	}
	return true;
}
#endif

#endif // WITH_DEV_AUTOMATION_TESTS
