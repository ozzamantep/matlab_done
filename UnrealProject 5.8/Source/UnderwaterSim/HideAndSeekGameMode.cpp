// HideAndSeekGameMode.cpp
#include "HideAndSeekGameMode.h"
#include "Environment/UnderwaterLevelSetup.h"
#include "Sensors/SonarSensorComponent.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/PlayerController.h"
#include "Engine/Engine.h"
#include "Engine/StaticMeshActor.h"
#include "InputCoreTypes.h"
#include "UnderwaterSim.h"

AHideAndSeekGameMode::AHideAndSeekGameMode()
{
    PrimaryActorTick.bCanEverTick = true;
    DefaultPawnClass = AUnderwaterDrone::StaticClass();
}

void AHideAndSeekGameMode::BeginPlay()
{
    Super::BeginPlay();

    // Ensure the underwater world is populated immediately before finding drones
    TArray<AActor*> FoundSetups;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), AUnderwaterLevelSetup::StaticClass(), FoundSetups);
    AUnderwaterLevelSetup* LevelSetup = nullptr;
    if (FoundSetups.Num() == 0)
    {
        FActorSpawnParameters SpawnParams;
        SpawnParams.SpawnCollisionHandlingOverride = ESpawnActorCollisionHandlingMethod::AlwaysSpawn;
        LevelSetup = GetWorld()->SpawnActor<AUnderwaterLevelSetup>(
            AUnderwaterLevelSetup::StaticClass(), FVector::ZeroVector, FRotator::ZeroRotator, SpawnParams);
    }
    else
    {
        LevelSetup = Cast<AUnderwaterLevelSetup>(FoundSetups[0]);
    }

    if (LevelSetup)
    {
        LevelSetup->PopulateLevel();
    }

    FindDrones();

    // Auto-possess the Seeker drone so the player controls it directly
    if (SeekerDrone)
    {
        APlayerController* PC = GetWorld()->GetFirstPlayerController();
        if (PC)
        {
            APawn* OldPawn = PC->GetPawn();
            if (OldPawn && OldPawn != SeekerDrone)
            {
                PC->UnPossess();
                OldPawn->Destroy();
            }
            PC->Possess(SeekerDrone);
            PC->SetControlRotation(FRotator(-15.0f, SeekerDrone->GetActorRotation().Yaw, 0.0f));
            UE_LOG(LogUnderwaterSim, Log, TEXT("Player now possessing SEEKER drone (%s)!"), *SeekerDrone->GetName());
        }
    }

    StartNewMatch();
}

void AHideAndSeekGameMode::FindDrones()
{
    TArray<AActor*> FoundDrones;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), AUnderwaterDrone::StaticClass(), FoundDrones);

    for (AActor* Actor : FoundDrones)
    {
        AUnderwaterDrone* Drone = Cast<AUnderwaterDrone>(Actor);
        if (!Drone) continue;

        const FString Name = Drone->GetName();
        if ((Drone->DroneRole == EDroneRole::Seeker || Name.Contains(TEXT("Seeker"))) && !SeekerDrone)
        {
            SeekerDrone = Drone;
            SeekerDrone->DroneRole = EDroneRole::Seeker;
        }
        else if ((Drone->DroneRole == EDroneRole::Hider || Name.Contains(TEXT("Hider"))) && !HiderDrone)
        {
            HiderDrone = Drone;
            HiderDrone->DroneRole = EDroneRole::Hider;
        }
    }

    // Fallback: if still no SeekerDrone, pick the first available drone
    if (!SeekerDrone && FoundDrones.Num() > 0)
    {
        SeekerDrone = Cast<AUnderwaterDrone>(FoundDrones[0]);
        if (SeekerDrone) SeekerDrone->DroneRole = EDroneRole::Seeker;
    }
}

void AHideAndSeekGameMode::FindHidingCovers()
{
    HidingCoverActors.Empty();
    SearchWaypoints.Empty();

    if (!GetWorld()) return;

    TArray<AActor*> MeshActors;
    UGameplayStatics::GetAllActorsOfClass(GetWorld(), AStaticMeshActor::StaticClass(), MeshActors);

    for (AActor* Actor : MeshActors)
    {
        if (!Actor) continue;
        const FString Name = Actor->GetName();
        // Skip sky, ground, road, water planes
        if (Name.Contains(TEXT("Sky")) || Name.Contains(TEXT("Floor")) ||
            Name.Contains(TEXT("Road")) || Name.Contains(TEXT("Terrain")) ||
            Name.Contains(TEXT("Water")))
        {
            continue;
        }

        FVector Origin, Extent;
        Actor->GetActorBounds(true, Origin, Extent);
        const float MaxExtent = FMath::Max(Extent.X, Extent.Y);
        if (MaxExtent >= 180.0f && MaxExtent <= 2500.0f && Extent.Z >= 150.0f)
        {
            HidingCoverActors.Add(Actor);
            // Add a patrol waypoint near this building at underwater cruise height
            const float PatrolZ = FMath::Clamp(Origin.Z + 120.0f, -1500.0f, -450.0f);
            SearchWaypoints.Add(FVector(Origin.X + MaxExtent + 320.0f, Origin.Y, PatrolZ));
        }
    }

    // Fallback waypoints if no buildings found
    if (SearchWaypoints.Num() == 0)
    {
        SearchWaypoints = {
            FVector(-2500.0f, -2500.0f, -800.0f),
            FVector( 2500.0f, -2500.0f, -900.0f),
            FVector( 2500.0f,  2500.0f, -800.0f),
            FVector(-2500.0f,  2500.0f, -900.0f)
        };
    }

    // Shuffle search waypoints so Seeker's patrol is natural and non-monotonous
    for (int32 i = SearchWaypoints.Num() - 1; i > 0; --i)
    {
        const int32 j = FMath::RandRange(0, i);
        SearchWaypoints.Swap(i, j);
    }

    UE_LOG(LogUnderwaterSim, Log, TEXT("Found %d building/cover obstacles for Hide-and-Seek!"), HidingCoverActors.Num());
}

FVector AHideAndSeekGameMode::PickBestHidingSpot() const
{
    if (!SeekerDrone || !HidingCoverActors.Num())
    {
        return FVector(FMath::RandRange(-2500.0f, 2500.0f), FMath::RandRange(-2500.0f, 2500.0f), -800.0f);
    }

    const FVector SeekerPos = SeekerDrone->GetActorLocation();
    const FVector HiderPos  = HiderDrone ? HiderDrone->GetActorLocation() : SeekerPos;

    TArray<FVector> OccludedSpots;
    TArray<FVector> FallbackSpots;
    for (AActor* Cover : HidingCoverActors)
    {
        if (!Cover) continue;
        FVector Origin, Extent;
        Cover->GetActorBounds(true, Origin, Extent);
        const float Radius = FMath::Max(Extent.X, Extent.Y);

        // Spot tucked close behind the opposite side of the building from Seeker
        FVector AwayFromSeeker = (Origin - SeekerPos);
        AwayFromSeeker.Z = 0.0f;
        if (AwayFromSeeker.IsNearlyZero()) AwayFromSeeker = FVector(1.0f, 0.0f, 0.0f);
        AwayFromSeeker.Normalize();

        FVector Spot = Origin + AwayFromSeeker * (Radius + 160.0f);
        Spot.Z = FMath::Clamp(Origin.Z + 60.0f, -1350.0f, -450.0f);

        const float DistToSeeker = FVector::Dist(Spot, SeekerPos);
        const float DistToOld    = FVector::Dist(Spot, CurrentHidingSpot);
        if (DistToSeeker > 1800.0f && DistToOld > 700.0f)
        {
            FHitResult Hit;
            FCollisionQueryParams Params;
            if (SeekerDrone) Params.AddIgnoredActor(SeekerDrone);
            if (HiderDrone)  Params.AddIgnoredActor(HiderDrone);
            // Check if a building actually blocks line-of-sight to Seeker!
            const bool bBlocked = GetWorld() && GetWorld()->LineTraceSingleByChannel(
                Hit, Spot, SeekerPos, ECC_Visibility, Params);
            if (bBlocked)
            {
                OccludedSpots.Add(Spot);
            }
            else
            {
                FallbackSpots.Add(Spot);
            }
        }
    }

    if (OccludedSpots.Num() > 0)
    {
        return OccludedSpots[FMath::RandRange(0, OccludedSpots.Num() - 1)];
    }
    if (FallbackSpots.Num() > 0)
    {
        return FallbackSpots[FMath::RandRange(0, FallbackSpots.Num() - 1)];
    }

    // Fallback: pick any building cover at random
    AActor* RandomCover = HidingCoverActors[FMath::RandRange(0, HidingCoverActors.Num() - 1)];
    FVector Origin, Extent;
    RandomCover->GetActorBounds(true, Origin, Extent);
    return FVector(Origin.X + Extent.X + 320.0f, Origin.Y, FMath::Clamp(Origin.Z, -1400.0f, -450.0f));
}

FVector AHideAndSeekGameMode::PickBuildingOrbitSpot() const
{
    if (!HiderDrone || !SeekerDrone || HidingCoverActors.Num() == 0)
    {
        return PickBestHidingSpot();
    }

    const FVector HiderPos = HiderDrone->GetActorLocation();
    const FVector SeekerPos = SeekerDrone->GetActorLocation();

    // Find nearest building to Hider so it can loop around it and break line-of-sight
    AActor* NearestCover = nullptr;
    float BestDist = FLT_MAX;
    for (AActor* Cover : HidingCoverActors)
    {
        if (!Cover) continue;
        const float D = FVector::Dist(HiderPos, Cover->GetActorLocation());
        if (D < BestDist)
        {
            BestDist = D;
            NearestCover = Cover;
        }
    }

    if (!NearestCover) return PickBestHidingSpot();

    FVector Origin, Extent;
    NearestCover->GetActorBounds(true, Origin, Extent);
    const float Radius = FMath::Max(Extent.X, Extent.Y) + 190.0f;

    // Orbit 110 degrees around the building away from Seeker
    FVector FromCenterToHider = (HiderPos - Origin).GetSafeNormal2D();
    if (FromCenterToHider.IsNearlyZero()) FromCenterToHider = FVector(1.0f, 0.0f, 0.0f);
    const FVector OrbitDir = FromCenterToHider.RotateAngleAxis(110.0f * HiderJukeSign, FVector::UpVector);

    FVector OrbitSpot = Origin + OrbitDir * Radius;
    OrbitSpot.Z = FMath::Clamp(Origin.Z + 70.0f, -1350.0f, -450.0f);
    return OrbitSpot;
}

void AHideAndSeekGameMode::StartNewMatch()
{
    RoundTimer = 0.0f;
    CurrentPhase = EGameStatePhase::HidingPhase;
    bHasDetectedHider = false;
    TimeSinceLastDetection = 999.0f;
    SeekerAIState = TEXT("IDLE");
    CurrentWaypointIdx = 0;
    RetargetCoverTimer = 0.0f;

    FindHidingCovers();
    CurrentHidingSpot = PickBestHidingSpot();

    OnGamePhaseChanged.Broadcast(CurrentPhase);

    UE_LOG(LogUnderwaterSim, Log, TEXT("Game started! Phase: HIDING (Headstart: %.1fs)"), HidingHeadstartSeconds);
}

void AHideAndSeekGameMode::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    if (CurrentPhase == EGameStatePhase::SeekerWon || CurrentPhase == EGameStatePhase::HiderWon)
    {
        if (SeekerDrone) SeekerDrone->SetAutonomousVelocity(FVector::ZeroVector, SeekerDrone->GetActorRotation().Yaw);
        if (HiderDrone)  HiderDrone->SetAutonomousVelocity(FVector::ZeroVector, HiderDrone->GetActorRotation().Yaw);

        if (GEngine)
        {
            const FString WinText = (CurrentPhase == EGameStatePhase::SeekerWon)
                ? FString::Printf(TEXT("MATCH OVER: SEEKER CAPTURED HIDER in %.1fs! (Dist: %.2fm)"), RoundTimer, CurrentDistanceBetweenDrones)
                : FString::Printf(TEXT("MATCH OVER: HIDER SURVIVED %.1fs!"), RoundTimer);
            GEngine->AddOnScreenDebugMessage(1, 0.1f, FColor::Green, WinText);
        }
        return;
    }

    RoundTimer += DeltaTime;

    if (CurrentPhase == EGameStatePhase::HidingPhase && RoundTimer >= HidingHeadstartSeconds)
    {
        CurrentPhase = EGameStatePhase::SeekingPhase;
        OnGamePhaseChanged.Broadcast(CurrentPhase);
        UE_LOG(LogUnderwaterSim, Log, TEXT("Phase Changed: ACTIVE SEEKING!"));
    }

    if (RoundTimer >= MaxRoundDurationSeconds)
    {
        CurrentPhase = EGameStatePhase::HiderWon;
        OnGamePhaseChanged.Broadcast(CurrentPhase);
        UE_LOG(LogUnderwaterSim, Log, TEXT("Match Over: HIDER SURVIVED!"));
        return;
    }

    if (bEnableAutonomousAI)
    {
        UpdateAutonomousAI(DeltaTime);
    }

    UpdateCinematicPOV(DeltaTime);

    if (CurrentPhase == EGameStatePhase::SeekingPhase)
    {
        CheckCaptureCondition();
    }

    if (GEngine)
    {
        const FString PhaseStr = (CurrentPhase == EGameStatePhase::HidingPhase) ? TEXT("HIDING (Headstart)") : TEXT("SEEKING");
        const FString POVStr = (CameraPOVMode == 0) ? TEXT("Action Chase") :
                               (CameraPOVMode == 1) ? TEXT("Hider Evasion") : TEXT("Wide Tactical");
        const FString HUD = FString::Printf(
            TEXT("Hide & Seek | %s | AI: %s | Time: %.1fs | Dist: %.2fm | POV [V]: %s"),
            *PhaseStr, *SeekerAIState, RoundTimer, CurrentDistanceBetweenDrones, *POVStr);
        GEngine->AddOnScreenDebugMessage(1, 0.1f, FColor::Cyan, HUD);
    }
}

void AHideAndSeekGameMode::UpdateCinematicPOV(float DeltaTime)
{
    APlayerController* PC = GetWorld() ? GetWorld()->GetFirstPlayerController() : nullptr;
    if (!PC || !SeekerDrone || !HiderDrone) return;

    const bool bVDown = PC->IsInputKeyDown(EKeys::V);
    if (bVDown && !bPrevVKeyDown)
    {
        CameraPOVMode = (CameraPOVMode + 1) % 3;
        AUnderwaterDrone* TargetPawn = (CameraPOVMode == 1) ? HiderDrone : SeekerDrone;
        if (PC->GetPawn() != TargetPawn)
        {
            PC->Possess(TargetPawn);
        }
    }
    bPrevVKeyDown = bVDown;

    AUnderwaterDrone* ActiveDrone = Cast<AUnderwaterDrone>(PC->GetPawn());
    if (!ActiveDrone) return;

    // Smoothly steer camera to frame the chase unless player moves mouse
    float MouseX = 0.0f, MouseY = 0.0f;
    PC->GetInputMouseDelta(MouseX, MouseY);
    const bool bMouseMoving = !FMath::IsNearlyZero(MouseX, 0.05f) || !FMath::IsNearlyZero(MouseY, 0.05f);

    if (!bMouseMoving)
    {
        FRotator DesiredCamRot = ActiveDrone->GetActorRotation();
        if (CameraPOVMode == 0 && (SeekerAIState == TEXT("DETECTED") || SeekerAIState == TEXT("APPROACHING")))
        {
            // Frame both Seeker and Hider in the chase!
            const FVector ToHider = (HiderDrone->GetActorLocation() - SeekerDrone->GetActorLocation()).GetSafeNormal();
            DesiredCamRot = ToHider.Rotation();
        }
        else if (CameraPOVMode == 1)
        {
            // Hider POV: angle slightly toward Seeker so you see the pursuer chasing you
            const FVector ToSeeker = (SeekerDrone->GetActorLocation() - HiderDrone->GetActorLocation()).GetSafeNormal();
            DesiredCamRot = FMath::RInterpTo(ActiveDrone->GetActorRotation(), ToSeeker.Rotation(), 0.35f, 1.0f);
        }

        DesiredCamRot.Pitch = (CameraPOVMode == 2) ? -32.0f : -16.0f;
        DesiredCamRot.Roll = 0.0f;
        PC->SetControlRotation(FMath::RInterpTo(PC->GetControlRotation(), DesiredCamRot, DeltaTime, 3.2f));
    }

    const float TargetArm = (CameraPOVMode == 2) ? 850.0f :
                            (SeekerAIState == TEXT("APPROACHING")) ? 480.0f : 420.0f;
    const float TargetFOV = (SeekerAIState == TEXT("APPROACHING")) ? 102.0f : 92.0f;
    ActiveDrone->UpdateCinematicCamera(TargetArm, TargetFOV, DeltaTime);
}

FVector AHideAndSeekGameMode::ComputeObstacleAvoidance(AUnderwaterDrone* Drone, const FVector& DesiredDir, float& TurnSign, float& TurnLockTimer, float DeltaTime) const
{
    if (!Drone || !GetWorld()) return DesiredDir;

    TurnLockTimer = FMath::Max(0.0f, TurnLockTimer - DeltaTime);

    const FVector Start = Drone->GetActorLocation();
    const float SweepDist = 320.0f; // Look ahead 3.2m with a sphere sweep so we steer EARLY and smoothly
    FCollisionQueryParams Params;
    Params.AddIgnoredActor(Drone);
    if (SeekerDrone) Params.AddIgnoredActor(SeekerDrone);
    if (HiderDrone)  Params.AddIgnoredActor(HiderDrone);

    FHitResult Hit;
    const FCollisionShape Sphere = FCollisionShape::MakeSphere(85.0f);
    const FVector End = Start + DesiredDir.GetSafeNormal() * SweepDist;

    if (GetWorld()->SweepSingleByChannel(Hit, Start, End, FQuat::Identity, ECC_Visibility, Sphere, Params))
    {
        // Lock turn direction for 1.5s so the drone NEVER oscillates left-right-left-right!
        if (TurnLockTimer <= 0.0f)
        {
            const FVector Right = FVector::CrossProduct(FVector::UpVector, DesiredDir).GetSafeNormal();
            TurnSign = (FVector::DotProduct(Hit.ImpactNormal, Right) >= 0.0f) ? 1.0f : -1.0f;
            TurnLockTimer = 1.5f;
        }

        const float Urgency = FMath::Clamp(1.0f - (Hit.Distance / SweepDist), 0.15f, 1.0f);
        const FVector SideDir = FVector::CrossProduct(FVector::UpVector, DesiredDir).GetSafeNormal() * TurnSign;
        // Steer smoothly sideways + slightly upward to glide over low arches/rocks
        FVector Steer = DesiredDir * (1.0f - 0.5f * Urgency) + SideDir * (1.35f * Urgency) + FVector(0.0f, 0.0f, 0.55f * Urgency);
        return Steer.GetSafeNormal();
    }

    // Soft depth & arena bounds guard
    FVector Guard = DesiredDir;
    if (Start.Z > -180.0f) Guard.Z -= 0.5f;
    if (Start.Z < -2400.0f) Guard.Z += 0.5f;
    if (Start.X >  5800.0f) Guard.X -= 0.8f;
    if (Start.X < -5800.0f) Guard.X += 0.8f;
    if (Start.Y >  5800.0f) Guard.Y -= 0.8f;
    if (Start.Y < -5800.0f) Guard.Y += 0.8f;

    return Guard.GetSafeNormal();
}

void AHideAndSeekGameMode::UpdateAutonomousAI(float DeltaTime)
{
    if (!SeekerDrone || !HiderDrone)
    {
        FindDrones();
        if (!SeekerDrone || !HiderDrone) return;
    }

    const FVector SeekerPos = SeekerDrone->GetActorLocation();
    const FVector HiderPos  = HiderDrone->GetActorLocation();
    CurrentDistanceBetweenDrones = FVector::Dist(SeekerPos, HiderPos) / 100.0f;

    // ── 1. Hider Autonomous Evasion, Cover & Juking ("Gocek") AI ──
    {
        RetargetCoverTimer += DeltaTime;
        HiderJukeTimer += DeltaTime;
        const float DistToCover = FVector::Dist(HiderPos, CurrentHidingSpot);

        // Check if Seeker currently has line-of-sight to Hider
        bool bSeekerCanSeeHider = false;
        if (CurrentPhase == EGameStatePhase::SeekingPhase && CurrentDistanceBetweenDrones <= 24.0f && GetWorld())
        {
            FHitResult SightHit;
            FCollisionQueryParams SightParams;
            SightParams.AddIgnoredActor(SeekerDrone);
            SightParams.AddIgnoredActor(HiderDrone);
            bSeekerCanSeeHider = !GetWorld()->LineTraceSingleByChannel(
                SightHit, SeekerPos, HiderPos, ECC_Visibility, SightParams);
        }

        const bool bInChase = (CurrentPhase == EGameStatePhase::SeekingPhase) &&
                              (bSeekerCanSeeHider || CurrentDistanceBetweenDrones < 18.0f);

        if (CurrentHidingSpot.IsZero() ||
            (bInChase && RetargetCoverTimer > 4.0f) ||
            RetargetCoverTimer > 18.0f)
        {
            CurrentHidingSpot = PickBestHidingSpot();
            RetargetCoverTimer = 0.0f;
        }

        FVector HiderDir = FVector::ZeroVector;
        float HiderSpeedCm = 0.0f;

        if (bInChase)
        {
            // Active "gocek-gocek" around nearest building + adrenaline burst to break line-of-sight and escape!
            if (HiderJukeTimer >= 1.4f)
            {
                HiderJukeSign = -HiderJukeSign;
                HiderJukeTimer = 0.0f;
            }

            const FVector OrbitSpot = PickBuildingOrbitSpot();
            const FVector ToOrbit = (OrbitSpot - HiderPos).GetSafeNormal();
            const FVector Away = (HiderPos - SeekerPos).GetSafeNormal2D();
            const FVector Side(-Away.Y, Away.X, 0.0f);
            const float JukeWeight = (CurrentDistanceBetweenDrones < 12.0f) ? 1.25f : 0.75f;
            const float VertJuke = 0.35f * FMath::Sin(RoundTimer * 1.8f);

            HiderDir = (ToOrbit * 0.85f + Away * 0.65f + Side * (HiderJukeSign * JukeWeight) + FVector(0.0f, 0.0f, VertJuke)).GetSafeNormal();
            HiderDir = ComputeObstacleAvoidance(HiderDrone, HiderDir, HiderTurnSign, HiderTurnLockTimer, DeltaTime);
            // Adrenaline burst when Seeker gets close so Hider can genuinely juke & escape!
            HiderSpeedCm = (CurrentDistanceBetweenDrones < 12.0f) ? 575.0f : 540.0f;
        }
        else if (DistToCover > 200.0f)
        {
            HiderDir = (CurrentHidingSpot - HiderPos).GetSafeNormal();
            HiderDir = ComputeObstacleAvoidance(HiderDrone, HiderDir, HiderTurnSign, HiderTurnLockTimer, DeltaTime);
            HiderSpeedCm = (CurrentPhase == EGameStatePhase::HidingPhase) ? 620.0f : 480.0f;
        }
        else
        {
            // Tucked behind building! Gently hover
            HiderSpeedCm = 30.0f;
            HiderDir = (HiderPos - SeekerPos).GetSafeNormal2D();
        }

        const float HiderYaw = HiderDir.IsNearlyZero() ? HiderDrone->GetActorRotation().Yaw : HiderDir.Rotation().Yaw;
        HiderDrone->SetAutonomousVelocity(HiderDir * HiderSpeedCm, HiderYaw);
    }

    // ── 2. Seeker Autonomous Hide-and-Seek AI ──
    if (CurrentPhase == EGameStatePhase::HidingPhase)
    {
        SeekerAIState = TEXT("IDLE");
        const float ScanYaw = SeekerDrone->GetActorRotation().Yaw + 25.0f * DeltaTime;
        SeekerDrone->SetAutonomousVelocity(FVector::ZeroVector, ScanYaw);
        return;
    }

    // Check Sonar / Line-of-Sight detection of Hider
    TimeSinceLastDetection += DeltaTime;
    bool bDetectedThisTick = false;

    if (USonarSensorComponent* Sonar = SeekerDrone->FindComponentByClass<USonarSensorComponent>())
    {
        for (const FSonarDetection& Det : Sonar->GetLatestDetections())
        {
            if (Det.DetectedActor == HiderDrone)
            {
                bDetectedThisTick = true;
                break;
            }
        }
    }

    // Line-of-sight detection only within 22m (so Seeker truly has to search around buildings!)
    if (!bDetectedThisTick && CurrentDistanceBetweenDrones <= 22.0f && GetWorld())
    {
        FHitResult Hit;
        FCollisionQueryParams Params;
        Params.AddIgnoredActor(SeekerDrone);
        if (!GetWorld()->LineTraceSingleByChannel(Hit, SeekerPos, HiderPos, ECC_Visibility, Params) ||
            Hit.GetActor() == HiderDrone)
        {
            bDetectedThisTick = true;
        }
    }

    if (bDetectedThisTick)
    {
        LastKnownHiderPos = HiderPos;
        TimeSinceLastDetection = 0.0f;
        bHasDetectedHider = true;
    }

    // State Machine: SEARCHING -> DETECTED -> APPROACHING -> LOST -> FOUND
    // Short 2.2s memory window so if Hider loops behind a building and breaks sight, Hider actually escapes!
    if (CurrentDistanceBetweenDrones <= CaptureDistanceThresholdMeters)
    {
        SeekerAIState = TEXT("FOUND");
    }
    else if (bHasDetectedHider && TimeSinceLastDetection < 2.2f)
    {
        SeekerAIState = (CurrentDistanceBetweenDrones < 18.0f) ? TEXT("APPROACHING") : TEXT("DETECTED");
    }
    else if (bHasDetectedHider && TimeSinceLastDetection < 5.0f)
    {
        SeekerAIState = TEXT("LOST");
    }
    else
    {
        SeekerAIState = TEXT("SEARCHING");
    }

    FVector GoalPos = LastKnownHiderPos;
    float SeekerSpeedCm = 520.0f;

    if (SeekerAIState == TEXT("APPROACHING"))
    {
        SeekerSpeedCm = 555.0f;
        const FVector HiderVel = HiderDrone->GetVelocityMetersPerSec() * 100.0f;
        GoalPos = LastKnownHiderPos + HiderVel * 0.35f;
    }
    else if (SeekerAIState == TEXT("DETECTED"))
    {
        SeekerSpeedCm = 530.0f;
    }
    else if (SeekerAIState == TEXT("SEARCHING") || SeekerAIState == TEXT("LOST"))
    {
        SeekerSpeedCm = 460.0f;
        if (SeekerAIState == TEXT("SEARCHING") && SearchWaypoints.Num() > 0)
        {
            // Anti-stuck detector: if Seeker hasn't progressed > 120cm in 1.5s, skip to next waypoint
            SeekerStuckTimer += DeltaTime;
            if (SeekerStuckTimer >= 1.5f)
            {
                if (FVector::Dist(SeekerPos, LastSeekerCheckPos) < 120.0f)
                {
                    CurrentWaypointIdx = (CurrentWaypointIdx + 1) % SearchWaypoints.Num();
                }
                LastSeekerCheckPos = SeekerPos;
                SeekerStuckTimer = 0.0f;
            }

            GoalPos = SearchWaypoints[CurrentWaypointIdx];
            if (FVector::Dist(SeekerPos, GoalPos) < 550.0f)
            {
                CurrentWaypointIdx = (CurrentWaypointIdx + 1) % SearchWaypoints.Num();
                GoalPos = SearchWaypoints[CurrentWaypointIdx];
            }
        }
    }

    FVector SeekerDir = (GoalPos - SeekerPos).GetSafeNormal();
    SeekerDir = ComputeObstacleAvoidance(SeekerDrone, SeekerDir, SeekerTurnSign, SeekerTurnLockTimer, DeltaTime);
    const float SeekerYaw = SeekerDir.Rotation().Yaw;
    SeekerDrone->SetAutonomousVelocity(SeekerDir * SeekerSpeedCm, SeekerYaw);
}

void AHideAndSeekGameMode::CheckCaptureCondition()
{
    if (!SeekerDrone || !HiderDrone)
    {
        FindDrones();
        return;
    }

    const float DistanceCm = FVector::Dist(SeekerDrone->GetActorLocation(), HiderDrone->GetActorLocation());
    CurrentDistanceBetweenDrones = DistanceCm / 100.0f;

    if (CurrentDistanceBetweenDrones <= CaptureDistanceThresholdMeters)
    {
        CurrentPhase = EGameStatePhase::SeekerWon;
        OnGamePhaseChanged.Broadcast(CurrentPhase);
        OnTargetCaptured.Broadcast(SeekerDrone, HiderDrone);
        UE_LOG(LogUnderwaterSim, Log, TEXT("Match Over: SEEKER CAPTURED HIDER at distance %.2fm!"), CurrentDistanceBetweenDrones);
    }
}
