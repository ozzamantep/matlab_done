// HideAndSeekGameMode.cpp
#include "HideAndSeekGameMode.h"
#include "Environment/UnderwaterLevelSetup.h"
#include "Sensors/SonarSensorComponent.h"
#include "Kismet/GameplayStatics.h"
#include "GameFramework/PlayerController.h"
#include "Engine/Engine.h"
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

void AHideAndSeekGameMode::StartNewMatch()
{
    RoundTimer = 0.0f;
    CurrentPhase = EGameStatePhase::HidingPhase;
    bHasDetectedHider = false;
    TimeSinceLastDetection = 999.0f;
    SeekerAIState = TEXT("IDLE");
    CurrentWaypointIdx = 0;

    // 3D Lawnmower patrol waypoints around the underwater arena (UE cm, centered around origin)
    SearchWaypoints = {
        FVector(-3500.0f, -3500.0f, -900.0f),
        FVector( 3500.0f, -3500.0f, -1100.0f),
        FVector( 3500.0f,  -800.0f, -1300.0f),
        FVector(-3500.0f,  -800.0f, -1100.0f),
        FVector(-3500.0f,  1800.0f, -1300.0f),
        FVector( 3500.0f,  1800.0f, -1100.0f),
        FVector( 3500.0f,  3800.0f, -900.0f),
        FVector(-3500.0f,  3800.0f, -1100.0f),
        FVector(    0.0f,     0.0f, -1400.0f)
    };

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

    if (CurrentPhase == EGameStatePhase::SeekingPhase)
    {
        CheckCaptureCondition();
    }

    if (GEngine)
    {
        const FString PhaseStr = (CurrentPhase == EGameStatePhase::HidingPhase) ? TEXT("HIDING (Headstart)") : TEXT("SEEKING");
        const FString HUD = FString::Printf(
            TEXT("Hide & Seek | Phase: %s | Seeker AI: %s | Time: %.1fs / %.0fs | Dist: %.2fm"),
            *PhaseStr, *SeekerAIState, RoundTimer, MaxRoundDurationSeconds, CurrentDistanceBetweenDrones);
        GEngine->AddOnScreenDebugMessage(1, 0.1f, FColor::Cyan, HUD);
    }
}

FVector AHideAndSeekGameMode::ComputeObstacleAvoidance(AUnderwaterDrone* Drone, const FVector& DesiredDir) const
{
    if (!Drone || !GetWorld()) return DesiredDir;

    const FVector Start = Drone->GetActorLocation();
    const float TraceDist = 450.0f; // 4.5 meters
    FCollisionQueryParams Params;
    Params.AddIgnoredActor(Drone);

    FVector Repulsion = FVector::ZeroVector;
    const TArray<FVector> ProbeDirs = {
        DesiredDir,
        DesiredDir.RotateAngleAxis(35.0f, FVector::UpVector),
        DesiredDir.RotateAngleAxis(-35.0f, FVector::UpVector),
        FVector::UpVector,
        -FVector::UpVector
    };

    for (const FVector& Dir : ProbeDirs)
    {
        FHitResult Hit;
        if (GetWorld()->LineTraceSingleByChannel(Hit, Start, Start + Dir.GetSafeNormal() * TraceDist, ECC_Visibility, Params))
        {
            if (Cast<AUnderwaterDrone>(Hit.GetActor()) == nullptr)
            {
                const float Strength = 1.0f - (Hit.Distance / TraceDist);
                Repulsion += Hit.ImpactNormal * (Strength * 1.8f);
            }
        }
    }

    // Soft depth & arena bounds guard
    if (Start.Z > -150.0f) Repulsion.Z -= 1.0f;
    if (Start.Z < -2600.0f) Repulsion.Z += 1.0f;
    if (Start.X >  5500.0f) Repulsion.X -= 1.2f;
    if (Start.X < -5500.0f) Repulsion.X += 1.2f;
    if (Start.Y >  5500.0f) Repulsion.Y -= 1.2f;
    if (Start.Y < -5500.0f) Repulsion.Y += 1.2f;

    return (DesiredDir + Repulsion).GetSafeNormal();
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

    // ── 1. Hider Autonomous Evasion AI ──
    {
        FVector AwayDir = (HiderPos - SeekerPos).GetSafeNormal();
        if (AwayDir.IsNearlyZero()) AwayDir = FVector(-1.0f, -1.0f, 0.0f);

        const FVector Tangential(-AwayDir.Y, AwayDir.X, 0.0f);
        const float Weave = 0.55f * FMath::Sin(RoundTimer * 0.45f);
        FVector HiderDir = (AwayDir + Tangential * Weave + FVector(0.0f, 0.0f, 0.2f * FMath::Sin(RoundTimer * 0.35f))).GetSafeNormal();
        HiderDir = ComputeObstacleAvoidance(HiderDrone, HiderDir);

        const float HiderSpeedCm = (CurrentPhase == EGameStatePhase::HidingPhase) ? 380.0f : 310.0f;
        const float HiderYaw = HiderDir.Rotation().Yaw;
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

    // Fallback acoustic/line-of-sight detection within 45m
    if (!bDetectedThisTick && CurrentDistanceBetweenDrones <= 45.0f && GetWorld())
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
    if (CurrentDistanceBetweenDrones <= CaptureDistanceThresholdMeters)
    {
        SeekerAIState = TEXT("FOUND");
    }
    else if (bHasDetectedHider && TimeSinceLastDetection < 6.0f)
    {
        SeekerAIState = (CurrentDistanceBetweenDrones < 20.0f) ? TEXT("APPROACHING") : TEXT("DETECTED");
    }
    else if (bHasDetectedHider && TimeSinceLastDetection < 10.0f)
    {
        SeekerAIState = TEXT("LOST");
    }
    else
    {
        SeekerAIState = TEXT("SEARCHING");
    }

    FVector GoalPos = LastKnownHiderPos;
    float SeekerSpeedCm = 420.0f; // 4.2 m/s (faster than Hider 3.1 m/s so capture succeeds)

    if (SeekerAIState == TEXT("SEARCHING") || SeekerAIState == TEXT("LOST"))
    {
        SeekerSpeedCm = 350.0f;
        if (SearchWaypoints.Num() > 0)
        {
            GoalPos = SearchWaypoints[CurrentWaypointIdx];
            if (FVector::Dist(SeekerPos, GoalPos) < 500.0f)
            {
                CurrentWaypointIdx = (CurrentWaypointIdx + 1) % SearchWaypoints.Num();
                GoalPos = SearchWaypoints[CurrentWaypointIdx];
            }
        }
    }

    FVector SeekerDir = (GoalPos - SeekerPos).GetSafeNormal();
    SeekerDir = ComputeObstacleAvoidance(SeekerDrone, SeekerDir);
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
