#include "Config/ScenarioJson.h"
#include "Config/ScenarioConfig.h"

#include "Dom/JsonObject.h"
#include "Dom/JsonValue.h"
#include "HAL/FileManager.h"
#include "Misc/FileHelper.h"
#include "Serialization/JsonReader.h"
#include "Serialization/JsonSerializer.h"
#include "Serialization/JsonWriter.h"

namespace
{

TSharedPtr<FJsonObject> WriteSystem(const FSystemGenerationSetup& System)
{
	TSharedPtr<FJsonObject> Object = MakeShared<FJsonObject>();
	// Seed as a string: JSON numbers go through double and would corrupt uint64 seeds.
	Object->SetStringField(TEXT("Seed"), FString::Printf(TEXT("%llu"), System.Seed));
	Object->SetNumberField(TEXT("NumPlanets"), System.NumPlanets);
	Object->SetNumberField(TEXT("StarMass"), System.StarMass);
	Object->SetNumberField(TEXT("StarRadius"), System.StarRadius);
	Object->SetNumberField(TEXT("MinOrbitalRadius"), System.MinOrbitalRadius);
	Object->SetNumberField(TEXT("MaxOrbitalRadius"), System.MaxOrbitalRadius);
	Object->SetStringField(TEXT("BoundaryMode"),
		System.BoundaryMode == EBoundaryMode::FixedDistance ? TEXT("Fixed") : TEXT("Relative"));
	Object->SetNumberField(TEXT("FixedBoundaryDistance"), System.FixedBoundaryDistance);
	Object->SetNumberField(TEXT("BoundaryK"), System.BoundaryK);
	Object->SetBoolField(TEXT("bInclinations"), System.bInclinations);
	Object->SetNumberField(TEXT("MaxInclinationDeg"), System.MaxInclinationDeg);
	Object->SetNumberField(TEXT("PlanetMassMin"), System.PlanetMassMin);
	Object->SetNumberField(TEXT("PlanetMassMax"), System.PlanetMassMax);
	Object->SetNumberField(TEXT("PlanetRadiusMin"), System.PlanetRadiusMin);
	Object->SetNumberField(TEXT("PlanetRadiusMax"), System.PlanetRadiusMax);
	return Object;
}

void ReadSystem(const TSharedPtr<FJsonObject>& Object, FSystemGenerationSetup& System)
{
	System.Seed = static_cast<int64>(FCString::Strtoui64(*Object->GetStringField(TEXT("Seed")), nullptr, 10));
	System.NumPlanets = static_cast<int32>(Object->GetNumberField(TEXT("NumPlanets")));
	System.StarMass = Object->GetNumberField(TEXT("StarMass"));
	System.StarRadius = Object->GetNumberField(TEXT("StarRadius"));
	System.MinOrbitalRadius = Object->GetNumberField(TEXT("MinOrbitalRadius"));
	System.MaxOrbitalRadius = Object->GetNumberField(TEXT("MaxOrbitalRadius"));
	System.BoundaryMode = Object->GetStringField(TEXT("BoundaryMode")) == TEXT("Fixed")
		? EBoundaryMode::FixedDistance
		: EBoundaryMode::RelativeToOutermost;
	System.FixedBoundaryDistance = Object->GetNumberField(TEXT("FixedBoundaryDistance"));
	System.BoundaryK = Object->GetNumberField(TEXT("BoundaryK"));
	System.bInclinations = Object->GetBoolField(TEXT("bInclinations"));
	System.MaxInclinationDeg = Object->GetNumberField(TEXT("MaxInclinationDeg"));
	System.PlanetMassMin = Object->GetNumberField(TEXT("PlanetMassMin"));
	System.PlanetMassMax = Object->GetNumberField(TEXT("PlanetMassMax"));
	System.PlanetRadiusMin = Object->GetNumberField(TEXT("PlanetRadiusMin"));
	System.PlanetRadiusMax = Object->GetNumberField(TEXT("PlanetRadiusMax"));
}

TSharedPtr<FJsonObject> WriteSpacecraft(const FSpacecraftSetup& Spacecraft)
{
	TSharedPtr<FJsonObject> Object = MakeShared<FJsonObject>();
	Object->SetNumberField(TEXT("DryMass"), Spacecraft.DryMass);
	Object->SetNumberField(TEXT("FuelMass"), Spacecraft.FuelMass);
	Object->SetNumberField(TEXT("MaxThrust"), Spacecraft.MaxThrust);
	Object->SetNumberField(TEXT("Isp"), Spacecraft.Isp);
	Object->SetStringField(TEXT("StartingBodyId"), Spacecraft.StartingBodyId.ToString());
	Object->SetNumberField(TEXT("StartingOrbitRadius"), Spacecraft.StartingOrbitRadius);
	Object->SetNumberField(TEXT("InitialOrbitPhase"), Spacecraft.InitialOrbitPhase);
	Object->SetNumberField(TEXT("MaxFlightTime"), Spacecraft.MaxFlightTime);
	Object->SetNumberField(TEXT("MaxSpeed"), Spacecraft.MaxSpeed);
	return Object;
}

void ReadSpacecraft(const TSharedPtr<FJsonObject>& Object, FSpacecraftSetup& Spacecraft)
{
	Spacecraft.DryMass = Object->GetNumberField(TEXT("DryMass"));
	Spacecraft.FuelMass = Object->GetNumberField(TEXT("FuelMass"));
	Spacecraft.MaxThrust = Object->GetNumberField(TEXT("MaxThrust"));
	Spacecraft.Isp = Object->GetNumberField(TEXT("Isp"));
	Spacecraft.StartingBodyId = FName(*Object->GetStringField(TEXT("StartingBodyId")));
	Spacecraft.StartingOrbitRadius = Object->GetNumberField(TEXT("StartingOrbitRadius"));
	Spacecraft.InitialOrbitPhase = Object->GetNumberField(TEXT("InitialOrbitPhase"));
	Spacecraft.MaxFlightTime = Object->GetNumberField(TEXT("MaxFlightTime"));
	Spacecraft.MaxSpeed = Object->GetNumberField(TEXT("MaxSpeed"));
}

TSharedPtr<FJsonObject> WriteOptimization(const FOptimizationSetup& Optimization)
{
	TSharedPtr<FJsonObject> Object = MakeShared<FJsonObject>();
	Object->SetStringField(TEXT("Mode"),
		Optimization.Mode == EOptimizationMode::FuelEfficient ? TEXT("FuelEfficient")
		: Optimization.Mode == EOptimizationMode::TimeEfficient ? TEXT("TimeEfficient")
		: TEXT("Balanced"));

	TArray<TSharedPtr<FJsonValue>> Criteria;
	for (const FCriterionWeight& Criterion : Optimization.Criteria)
	{
		TSharedPtr<FJsonObject> CriterionObject = MakeShared<FJsonObject>();
		CriterionObject->SetStringField(TEXT("Id"), Criterion.CriterionId.ToString());
		CriterionObject->SetNumberField(TEXT("Weight"), Criterion.Weight);
		Criteria.Add(MakeShared<FJsonValueObject>(CriterionObject));
	}
	Object->SetArrayField(TEXT("Criteria"), MoveTemp(Criteria));

	Object->SetNumberField(TEXT("BeamWidth"), Optimization.BeamWidth);
	Object->SetNumberField(TEXT("MaxHops"), Optimization.MaxHops);
	Object->SetNumberField(TEXT("DepartureWindowSteps"), Optimization.DepartureWindowSteps);
	Object->SetNumberField(TEXT("FlightTimeSteps"), Optimization.FlightTimeSteps);
	return Object;
}

void ReadOptimization(const TSharedPtr<FJsonObject>& Object, FOptimizationSetup& Optimization)
{
	const FString Mode = Object->GetStringField(TEXT("Mode"));
	Optimization.Mode = Mode == TEXT("FuelEfficient") ? EOptimizationMode::FuelEfficient
		: Mode == TEXT("TimeEfficient") ? EOptimizationMode::TimeEfficient
		: EOptimizationMode::Balanced;

	Optimization.Criteria.Reset();
	for (const TSharedPtr<FJsonValue>& Value : Object->GetArrayField(TEXT("Criteria")))
	{
		const TSharedPtr<FJsonObject> CriterionObject = Value->AsObject();
		FCriterionWeight& Criterion = Optimization.Criteria.AddDefaulted_GetRef();
		Criterion.CriterionId = FName(*CriterionObject->GetStringField(TEXT("Id")));
		Criterion.Weight = CriterionObject->GetNumberField(TEXT("Weight"));
	}

	Optimization.BeamWidth = static_cast<int32>(Object->GetNumberField(TEXT("BeamWidth")));
	Optimization.MaxHops = static_cast<int32>(Object->GetNumberField(TEXT("MaxHops")));
	Optimization.DepartureWindowSteps = static_cast<int32>(Object->GetNumberField(TEXT("DepartureWindowSteps")));
	Optimization.FlightTimeSteps = static_cast<int32>(Object->GetNumberField(TEXT("FlightTimeSteps")));
}

} // namespace

namespace ScenarioJson
{

bool ExportToFile(const UScenarioConfig* Config, const FString& FilePath)
{
	if (Config == nullptr)
	{
		return false;
	}

	TSharedPtr<FJsonObject> Root = MakeShared<FJsonObject>();
	Root->SetObjectField(TEXT("System"), WriteSystem(Config->System));
	Root->SetObjectField(TEXT("Spacecraft"), WriteSpacecraft(Config->Spacecraft));
	Root->SetObjectField(TEXT("Optimization"), WriteOptimization(Config->Optimization));

	FString Json;
	TSharedRef<TJsonWriter<>> Writer = TJsonWriterFactory<>::Create(&Json);
	FJsonSerializer::Serialize(Root.ToSharedRef(), Writer);
	Writer->Close();

	if (!IFileManager::Get().MakeDirectory(*FPaths::GetPath(FilePath), true))
	{
		return false;
	}
	return FFileHelper::SaveStringToFile(Json, *FilePath);
}

UScenarioConfig* ImportFromFile(const FString& FilePath, UObject* Outer)
{
	FString Json;
	if (!FFileHelper::LoadFileToString(Json, *FilePath))
	{
		return nullptr;
	}

	TSharedRef<TJsonReader<>> Reader = TJsonReaderFactory<>::Create(Json);
	TSharedPtr<FJsonObject> Root;
	if (!FJsonSerializer::Deserialize(Reader, Root) || !Root.IsValid())
	{
		return nullptr;
	}

	UScenarioConfig* Config = NewObject<UScenarioConfig>(Outer);
	ReadSystem(Root->GetObjectField(TEXT("System")), Config->System);
	ReadSpacecraft(Root->GetObjectField(TEXT("Spacecraft")), Config->Spacecraft);
	ReadOptimization(Root->GetObjectField(TEXT("Optimization")), Config->Optimization);
	return Config;
}

} // namespace ScenarioJson
