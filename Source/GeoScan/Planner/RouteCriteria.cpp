#include "Planner/RouteCriteria.h"

namespace
{
	struct FCriterionEntry
	{
		FName Id;
		UClass* Class;
	};

	const TArray<FCriterionEntry>& GetCriterionRegistry()
	{
		static const TArray<FCriterionEntry> Registry = {
			{ FName(TEXT("Fuel")), URouteCriterion_Fuel::StaticClass() },
			{ FName(TEXT("Time")), URouteCriterion_Time::StaticClass() },
			{ FName(TEXT("Risk")), URouteCriterion_Risk::StaticClass() },
		};
		return Registry;
	}
}

URouteCriterion* URouteCriterion::CreateById(UObject* Outer, FName CriterionId)
{
	for (const FCriterionEntry& Entry : GetCriterionRegistry())
	{
		if (Entry.Id == CriterionId)
		{
			return NewObject<URouteCriterion>(Outer, Entry.Class);
		}
	}
	return nullptr;
}

bool URouteCriterion::IsKnownCriterion(FName CriterionId)
{
	for (const FCriterionEntry& Entry : GetCriterionRegistry())
	{
		if (Entry.Id == CriterionId)
		{
			return true;
		}
	}
	return false;
}
