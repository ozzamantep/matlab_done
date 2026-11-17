// HideAndSeekGameMode.h
// Manages the rules, round phases, capture conditions, and scoring for Underwater Hide-and-Seek.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "Drone/UnderwaterDrone.h"
#include "HideAndSeekGameMode.generated.h"

UENUM(BlueprintType)
enum class EGameStatePhase : uint8
{
    Preparation     UMETA(DisplayName = "Preparation"),
    HidingPhase     UMETA(DisplayName = "Hiding Phase (Hider Headstart)"),
    SeekingPhase    UMETA(DisplayName = "Seeking Phase (Active Search)"),
    SeekerWon       UMETA(DisplayName = "Seeker Won (Target Captured)"),
    HiderWon        UMETA(DisplayName = "Hider Won (Time Expired)")
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_OneParam(FOnGamePhaseChanged, EGameStatePhase, NewPhase);
DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(FOnTargetCaptured, AUnderwaterDrone*, Seeker, AUnderwaterDrone*, Hider);

UCLASS()
class UNDERWATERSIM_API AHideAndSeekGameMode : public AGameModeBase
{
    GENERATED_BODY()

public:
    AHideAndSeekGameMode();

    virtual void Tick(float DeltaTime) override;

    // ── Game Parameters ──

    /** Duration of the hiding headstart in seconds */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rules")
    float HidingHeadstartSeconds = 14.0f;

    /** Total round duration in seconds */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rules")
    float MaxRoundDurationSeconds = 180.0f;

    /** Distance threshold in meters for successful capture */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rules")
    float CaptureDistanceThresholdMeters = 2.2f;

    /** Enable autonomous AI for both Seeker and Hider immediately on Play */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Rules|AI")
    bool bEnableAutonomousAI = true;

    // ── Current State ──

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rules|State")
    EGameStatePhase CurrentPhase = EGameStatePhase::Preparation;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rules|State")
    float RoundTimer = 0.0f;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Rules|State")
    float CurrentDistanceBetweenDrones = 0.0f;

    // ── Events ──

    UPROPERTY(BlueprintAssignable, Category = "Rules|Events")
    FOnGamePhaseChanged OnGamePhaseChanged;

    UPROPERTY(BlueprintAssignable, Category = "Rules|Events")
    FOnTargetCaptured OnTargetCaptured;

    UFUNCTION(BlueprintCallable, Category = "Rules")
    void StartNewMatch();

protected:
    virtual void BeginPlay() override;

private:
    UPROPERTY()
    AUnderwaterDrone* SeekerDrone = nullptr;

    UPROPERTY()
    AUnderwaterDrone* HiderDrone = nullptr;

    void FindDrones();
    void FindHidingCovers();
    FVector PickBestHidingSpot() const;
    FVector PickBuildingOrbitSpot() const;
    void CheckCaptureCondition();
    void UpdateAutonomousAI(float DeltaTime);
    void UpdateCinematicPOV(float DeltaTime);
    FVector ComputeObstacleAvoidance(AUnderwaterDrone* Drone, const FVector& DesiredDir, float& TurnSign, float& TurnLockTimer, float DeltaTime) const;

    TArray<FVector> SearchWaypoints;
    TArray<AActor*> HidingCoverActors;
    FVector CurrentHidingSpot = FVector::ZeroVector;
    float RetargetCoverTimer = 0.0f;
    float SeekerTurnSign = 1.0f;
    float SeekerTurnLockTimer = 0.0f;
    float HiderTurnSign = 1.0f;
    float HiderTurnLockTimer = 0.0f;
    float HiderJukeSign = 1.0f;
    float HiderBrakeTimer = 0.0f;
    float HiderBrakeCooldown = 0.0f;
    int32 CurrentWaypointIdx = 0;
    FVector LastKnownHiderPos = FVector::ZeroVector;
    FVector LastSeekerCheckPos = FVector::ZeroVector;
    float SeekerStuckTimer = 0.0f;
    float TimeSinceLastDetection = 999.0f;
    bool bHasDetectedHider = false;
    FString SeekerAIState = TEXT("IDLE");
    int32 CameraPOVMode = 0; // 0: Cinematic Chase, 1: Hider POV, 2: Wide Tactical
    bool bPrevVKeyDown = false;
};

