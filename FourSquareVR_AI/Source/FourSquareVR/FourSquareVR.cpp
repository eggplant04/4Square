// Fill out your copyright notice in the Description page of Project Settings.

#include "FourSquareVR.h"
#include "Modules/ModuleManager.h"
#include "Engine/Engine.h"
#include "Misc/CommandLine.h"
#include "Misc/CoreDelegates.h"
#include "Misc/Parse.h"

class FFourSquareVRModule : public FDefaultGameModuleImpl
{
	virtual void StartupModule() override
	{
#if !UE_BUILD_SHIPPING
		// Ephemeral, explicitly requested IP diagnostic. Never rewrites Steam/project configuration.
		if (FParse::Param(FCommandLine::Get(), TEXT("BallNetworkIPTest")))
			FCoreDelegates::OnPostEngineInit.AddLambda([]() {
				if (GEngine)
					for (auto &Definition : GEngine->NetDriverDefinitions)
						if (Definition.DefName == TEXT("GameNetDriver"))
							Definition.DriverClassName = Definition.DriverClassNameFallback =
								TEXT("/Script/OnlineSubsystemUtils.IpNetDriver");
			});
#endif
	}
};
IMPLEMENT_PRIMARY_GAME_MODULE(FFourSquareVRModule, FourSquareVR, "FourSquareVR");
