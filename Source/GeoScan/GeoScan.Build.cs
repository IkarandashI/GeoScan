// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class GeoScan : ModuleRules
{
	public GeoScan(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore", "EnhancedInput" });

		// Subfolder layout (Core/, Gen/, Planner/, ...): make headers includable as "Core/OrbitMath.h".
		PublicIncludePaths.Add(ModuleDirectory);

		PrivateDependencyModuleNames.AddRange(new string[] { "AssetRegistry", "Json" });

		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });
		
		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
	}
}
