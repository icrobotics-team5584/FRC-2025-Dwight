// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubFunnel.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "utilities/ICSpark.h"
#include "subsystems/SubEndEffector.h"
#include "utilities/RobotLogs.h"

SubFunnel::SubFunnel(){
    frc::SmartDashboard::PutData("SubFunnel/motorData", &_funnelMotor);
}

// This method will be called once per scheduler run
void SubFunnel::Periodic() {
}


frc2::CommandPtr SubFunnel::FeedDownFunnel() {
    return StartEnd([this] {_funnelMotor.Set(-0.8);}, [this] {_funnelMotor.Set(0);});
}

frc2::CommandPtr SubFunnel::FeedDownFunnelSLOW() {
    return StartEnd([this] {_funnelMotor.Set(-0.3);}, [this] {_funnelMotor.Set(0);});
}

frc2::CommandPtr SubFunnel::StopFunnelMotor(){
    return RunOnce([this] {_funnelMotor.Set(0);});
}


frc2::CommandPtr SubFunnel::FeedUpFunnel() {
    return StartEnd([this] {_funnelMotor.Set(0.3);}, [this] {_funnelMotor.Set(0);});
}