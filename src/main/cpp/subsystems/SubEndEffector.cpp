// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubEndEffector.h"


SubEndEffector::SubEndEffector() = default;

// This method will be called once per scheduler run
void SubEndEffector::Periodic() {}

frc2::CommandPtr SubEndEffector::EndEffectorStartMotor() {
    return RunOnce([this] {EndEffectorMotor.Set(0.5);});
}

frc2::CommandPtr SubEndEffector::EndEffectorReverseMotor() {
    return RunOnce([this] {EndEffectorMotor.Set(-0.5);});
}

frc2::CommandPtr SubEndEffector::EndEffectorStopMotor() {
    return RunOnce([this] {EndEffectorMotor.Set(0);});
}

frc2::CommandPtr SubEndEffector::EndEffectorIntake() {
    return EndEffectorStartMotor().AndThen(frc2::cmd::WaitUntil([this] {return CheckLineBreak();})).AndThen(EndEffectorStopMotor());
}

frc2::CommandPtr SubEndEffector::EndEffectorOuttake() {
    return EndEffectorReverseMotor().AndThen(frc2::cmd::WaitUntil([this] {return !CheckLineBreak();})).AndThen(EndEffectorStopMotor());
}


bool SubEndEffector::CheckLineBreak() {
    if(EndEffectorLineBreak.Get()) {
        return true;
    }
    else {
        return false;
    }
}
