// ROS2BridgeComponent.cpp
#include "ROS2Integration/ROS2BridgeComponent.h"
#include "Drone/UnderwaterDrone.h"
#include "Sensors/SonarSensorComponent.h"
#include "Sensors/IMUSensorComponent.h"
#include "Sensors/DepthSensorComponent.h"
#include "Sensors/DVLSensorComponent.h"
#include "Sockets.h"
#include "SocketSubsystem.h"
#include "Common/UdpSocketBuilder.h"
#include "UnderwaterSim.h"

UROS2BridgeComponent::UROS2BridgeComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void UROS2BridgeComponent::BeginPlay()
{
    Super::BeginPlay();

    OwnerDrone = Cast<AUnderwaterDrone>(GetOwner());
    if (OwnerDrone)
    {
        SonarComponent = OwnerDrone->FindComponentByClass<USonarSensorComponent>();
        IMUComponent = OwnerDrone->FindComponentByClass<UIMUSensorComponent>();
        DepthComponent = OwnerDrone->FindComponentByClass<UDepthSensorComponent>();
        DVLComponent = OwnerDrone->FindComponentByClass<UDVLSensorComponent>();

        InitUDPSockets();
        UE_LOG(LogUnderwaterSim, Log, TEXT("ROS2Bridge initialized for drone [%s] in namespace /%s (UDP %s:%d)"),
               *OwnerDrone->GetName(), *Namespace, *TelemetryTargetIP, TelemetryTargetPort);
    }
}

void UROS2BridgeComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    if (SendSocket)
    {
        SendSocket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(SendSocket);
        SendSocket = nullptr;
    }
    if (RecvSocket)
    {
        RecvSocket->Close();
        ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM)->DestroySocket(RecvSocket);
        RecvSocket = nullptr;
    }
    Super::EndPlay(EndPlayReason);
}

void UROS2BridgeComponent::InitUDPSockets()
{
    ISocketSubsystem* SocketSub = ISocketSubsystem::Get(PLATFORM_SOCKETSUBSYSTEM);
    if (!SocketSub) return;

    RemoteAddr = SocketSub->CreateInternetAddr();
    bool bValidIP = false;
    RemoteAddr->SetIp(*TelemetryTargetIP, bValidIP);
    RemoteAddr->SetPort(TelemetryTargetPort);

    SendSocket = FUdpSocketBuilder(TEXT("UE5_ROS2_TelemetrySender"))
        .AsNonBlocking()
        .AsReusable()
        .Build();

    const int32 ListenPort = (OwnerDrone && OwnerDrone->DroneRole == EDroneRole::Hider)
        ? (CmdVelListenPort + 1) : CmdVelListenPort;
    RecvSocket = FUdpSocketBuilder(TEXT("UE5_ROS2_CmdVelReceiver"))
        .AsNonBlocking()
        .AsReusable()
        .BoundToPort(ListenPort)
        .Build();
}

void UROS2BridgeComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    PollCmdVelUDP();

    OdomTimer += DeltaTime;
    const float Interval = 1.0f / FMath::Max(1.0f, OdomPublishRateHz);

    if (OdomTimer >= Interval)
    {
        OdomTimer = 0.0f;
        PublishOdometry();
    }
}

void UROS2BridgeComponent::PollCmdVelUDP()
{
    if (!RecvSocket || !OwnerDrone) return;

    uint32 PendingSize = 0;
    while (RecvSocket->HasPendingData(PendingSize))
    {
        TArray<uint8> Data;
        Data.SetNumUninitialized(FMath::Min(PendingSize, 1024u));
        int32 BytesRead = 0;
        if (RecvSocket->Recv(Data.GetData(), Data.Num(), BytesRead) && BytesRead > 0)
        {
            Data.Add(0);
            const FString Msg = UTF8_TO_TCHAR(reinterpret_cast<const char*>(Data.GetData()));
            // Format: "vx,vy,vz,yaw" (m/s or normalized)
            TArray<FString> Parts;
            Msg.ParseIntoArray(Parts, TEXT(","), true);
            if (Parts.Num() >= 4)
            {
                OnReceiveCmdVel(
                    FCString::Atof(*Parts[0]),
                    FCString::Atof(*Parts[1]),
                    FCString::Atof(*Parts[2]),
                    FCString::Atof(*Parts[3]));
            }
        }
    }
}

void UROS2BridgeComponent::OnReceiveCmdVel(float LinearX, float LinearY, float LinearZ, float AngularZ)
{
    if (!OwnerDrone) return;

    OwnerDrone->SetSurge(LinearX);
    OwnerDrone->SetSway(LinearY);
    OwnerDrone->SetHeave(LinearZ);
    OwnerDrone->SetYaw(AngularZ);
}

void UROS2BridgeComponent::OnReceiveEmergencyStop()
{
    if (OwnerDrone)
    {
        OwnerDrone->StopAllThrusters();
        UE_LOG(LogUnderwaterSim, Warning, TEXT("EMERGENCY STOP received for /%s"), *Namespace);
    }
}

void UROS2BridgeComponent::PublishOdometry()
{
    if (!OwnerDrone || !SendSocket || !RemoteAddr.IsValid()) return;

    // Convert Unreal Units (cm, left-handed Z-up) to ROS 2 standard (meters, right-handed REP 103)
    const FVector PosUE = OwnerDrone->GetActorLocation() / 100.0f;
    const FVector VelUE = OwnerDrone->GetVelocityMetersPerSec();
    const FRotator RotUE = OwnerDrone->GetActorRotation();

    const FString RoleStr = (OwnerDrone->DroneRole == EDroneRole::Hider) ? TEXT("hider") : TEXT("seeker");
    const FString JsonPayload = FString::Printf(
        TEXT("{\"role\":\"%s\",\"x\":%.3f,\"y\":%.3f,\"z\":%.3f,\"vx\":%.3f,\"vy\":%.3f,\"vz\":%.3f,\"yaw\":%.2f}"),
        *RoleStr, PosUE.X, -PosUE.Y, PosUE.Z, VelUE.X, -VelUE.Y, VelUE.Z, -RotUE.Yaw);

    FTCHARToUTF8 Converter(*JsonPayload);
    int32 BytesSent = 0;
    SendSocket->SendTo(reinterpret_cast<const uint8*>(Converter.Get()), Converter.Length(), BytesSent, *RemoteAddr);
}
