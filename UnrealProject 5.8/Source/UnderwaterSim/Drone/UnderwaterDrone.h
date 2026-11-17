// UnderwaterDrone.h
// Main Pawn class for the underwater drone (BlueROV2-inspired).

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "UnderwaterDrone.generated.h"

class UAUVBuoyancyComponent;
class UThrusterComponent;
class UStaticMeshComponent;
class UCameraComponent;
class USpringArmComponent;

/**
 * Drone role in the hide-and-seek game.
 */
UENUM(BlueprintType)
enum class EDroneRole : uint8
{
    Seeker   UMETA(DisplayName = "Seeker"),
    Hider    UMETA(DisplayName = "Hider"),
    Neutral  UMETA(DisplayName = "Neutral")
};

/**
 * AUnderwaterDrone
 *
 * A physics-based underwater drone pawn with:
 * - 6-DOF movement via vectored thrusters (BlueROV2 layout)
 * - Buoyancy and hydrodynamic drag
 * - Onboard camera and sensor attachment points
 * - ROS 2 interface for autonomous control
 */
UCLASS()
class UNDERWATERSIM_API AUnderwaterDrone : public APawn
{
    GENERATED_BODY()

public:
    AUnderwaterDrone();

    virtual void Tick(float DeltaTime) override;
    virtual void SetupPlayerInputComponent(class UInputComponent* PlayerInputComponent) override;

    // ── Components ──

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone")
    UStaticMeshComponent* DroneBody;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone")
    UStaticMeshComponent* VisualMesh;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone")
    UAUVBuoyancyComponent* BuoyancyComp;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Camera")
    USpringArmComponent* CameraBoom;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Camera")
    UCameraComponent* FollowCamera;

    // ── Thrusters (BlueROV2 Heavy: 6 thrusters) ──

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Thrusters")
    UThrusterComponent* ThrusterFL; // Front-Left horizontal

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Thrusters")
    UThrusterComponent* ThrusterFR; // Front-Right horizontal

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Thrusters")
    UThrusterComponent* ThrusterRL; // Rear-Left horizontal

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Thrusters")
    UThrusterComponent* ThrusterRR; // Rear-Right horizontal

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Thrusters")
    UThrusterComponent* ThrusterVL; // Vertical Left

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Thrusters")
    UThrusterComponent* ThrusterVR; // Vertical Right

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Propellers")
    USceneComponent* PropellerHubLeft;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Propellers")
    USceneComponent* PropellerHubRight;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Propellers")
    USceneComponent* PropellerHubFrontLeft;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Propellers")
    USceneComponent* PropellerHubFrontRight;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Propellers")
    USceneComponent* PropellerHubVertLeft;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Propellers")
    USceneComponent* PropellerHubVertRight;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Grabber")
    USceneComponent* ClawPivotLeft;

    UPROPERTY(VisibleAnywhere, BlueprintReadOnly, Category = "Drone|Grabber")
    USceneComponent* ClawPivotRight;

    // ── Game Properties ──

    /** Role of this drone in the hide-and-seek game */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Game")
    EDroneRole DroneRole = EDroneRole::Neutral;

    /** Unique drone ID for multi-agent identification */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Game")
    int32 DroneID = 0;

    /** When true, the drone moves autonomously according to Hide-and-Seek mission AI (unless player presses WASD) */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category = "Game|AI")
    bool bAutonomousAI = true;

    /** Drive the drone autonomously with a desired world-space velocity (cm/s) and heading (deg) */
    UFUNCTION(BlueprintCallable, Category = "Drone|Movement")
    void SetAutonomousVelocity(const FVector& WorldVelocityCmPerSec, float TargetYawDeg);

    /** Update camera boom arm length and FOV dynamically for cinematic chase feel */
    UFUNCTION(BlueprintCallable, Category = "Drone|Camera")
    void UpdateCinematicCamera(float TargetArmLength, float TargetFOV, float DeltaTime);

    /** Animate front robotic grabber claw (0.0 = closed/clamped, 1.0 = wide open) */
    UFUNCTION(BlueprintCallable, Category = "Drone|Grabber")
    void SetGrabberState(float TargetOpenAmount, float DeltaTime);

    // ── Movement API ──

    /** Move forward/backward (-1 to 1) */
    UFUNCTION(BlueprintCallable, Category = "Drone|Movement")
    void SetSurge(float Value);

    /** Move left/right (-1 to 1) */
    UFUNCTION(BlueprintCallable, Category = "Drone|Movement")
    void SetSway(float Value);

    /** Move up/down (-1 to 1) */
    UFUNCTION(BlueprintCallable, Category = "Drone|Movement")
    void SetHeave(float Value);

    /** Rotate yaw (-1 to 1) */
    UFUNCTION(BlueprintCallable, Category = "Drone|Movement")
    void SetYaw(float Value);

    /** Stop all thrusters */
    UFUNCTION(BlueprintCallable, Category = "Drone|Movement")
    void StopAllThrusters();

    /** Get current velocity in m/s */
    UFUNCTION(BlueprintPure, Category = "Drone|State")
    FVector GetVelocityMetersPerSec() const;

    /** Get current depth below water surface in meters */
    UFUNCTION(BlueprintPure, Category = "Drone|State")
    float GetCurrentDepth() const;

protected:
    virtual void BeginPlay() override;
    virtual void OnConstruction(const FTransform& Transform) override;

private:
    void SetupThrusters();

public:
    /** Apply the MBARI Doc Ricketts ROV visual mesh + role-based colors (Seeker = Yellow, Hider = Blue) */
    UFUNCTION(BlueprintCallable, Category = "Drone|Visual")
    void ApplyROVVisual();

private:
    void ApplyMixedThrusterInputs();

    void InputSurge(float Value);
    void InputSway(float Value);
    void InputHeave(float Value);
    void InputYaw(float Value);
    void InputLookUp(float Value);

    float PendingSurge = 0.0f;
    float PendingSway = 0.0f;
    float PendingHeave = 0.0f;
    float PendingYaw = 0.0f;

    FVector AutonomousWorldVelocity = FVector::ZeroVector;
    FVector SmoothedVelocity = FVector::ZeroVector;
    float AutonomousTargetYawDeg = 0.0f;
    bool bHasAutonomousCmd = false;

    // ── Propellers & Bubble Trail Pool ──
    struct FBubbleParticle
    {
        UStaticMeshComponent* Mesh = nullptr;
        FVector WorldPos = FVector::ZeroVector;
        FVector WorldVel = FVector::ZeroVector;
        float Age = 0.0f;
        float MaxLife = 0.8f;
        float BaseScale = 0.08f;
    };
    TArray<FBubbleParticle> BubblePool;
    int32 NextBubbleIdx = 0;
    float BubbleSpawnTimer = 0.0f;
    float CurrentClawOpen = 0.25f;

    void InitPropellersAndBubbles();
    void UpdatePropellersAndBubbles(float DeltaTime);
};
