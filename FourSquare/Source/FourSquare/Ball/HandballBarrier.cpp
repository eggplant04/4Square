// Fill out your copyright notice in the Description page of Project Settings.


#include "FourSquare/Ball/HandballBarrier.h"

// Sets default values
AHandballBarrier::AHandballBarrier()
{
 	// Set this actor to call Tick() every frame.  You can turn this off to improve performance if you don't need it.
	PrimaryActorTick.bCanEverTick = true;

}

// Called when the game starts or when spawned
void AHandballBarrier::BeginPlay()
{
	Super::BeginPlay();
	
}

// Called every frame
void AHandballBarrier::Tick(float DeltaTime)
{
	Super::Tick(DeltaTime);

}

