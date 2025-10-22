// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class FourSquare : ModuleRules
{
	public FourSquare(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

		PublicDependencyModuleNames.AddRange(new string[] {
			"Core",
			"CoreUObject",
			"Engine",
			"InputCore",
			"EnhancedInput",
			"AIModule",
			"StateTreeModule",
			"GameplayStateTreeModule",
			"UMG",
			"Slate"
		});

		PrivateDependencyModuleNames.AddRange(new string[] { });

		PublicIncludePaths.AddRange(new string[] {
			"FourSquare",
			"FourSquare/Variant_Platforming",
			"FourSquare/Variant_Platforming/Animation",
			"FourSquare/Variant_Combat",
			"FourSquare/Variant_Combat/AI",
			"FourSquare/Variant_Combat/Animation",
			"FourSquare/Variant_Combat/Gameplay",
			"FourSquare/Variant_Combat/Interfaces",
			"FourSquare/Variant_Combat/UI",
			"FourSquare/Variant_SideScrolling",
			"FourSquare/Variant_SideScrolling/AI",
			"FourSquare/Variant_SideScrolling/Gameplay",
			"FourSquare/Variant_SideScrolling/Interfaces",
			"FourSquare/Variant_SideScrolling/UI"
		});

		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });

		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
	}
}
