// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIntake.h"
#include <rev/SparkMax.h>

SubIntake::SubIntake() {
    rev::spark::SparkBaseConfig _intakeLeftConfig;
    _intakeLeftConfig.Follow(IntakeRight.GetDeviceId(), true);
    IntakeLeftMotor.AdjustConfig(_intakeLeftConfig);
  
}

// This method will be called once per scheduler run
void SubIntake::Periodic() {}

frc2::CommandPtr SubIntake::Intake() {
    return StartEnd([this] {IntakeRight.Set(0.5);}, [this] {IntakeRight.Set(0);});
}

frc2::CommandPtr SubIntake::Outtake() {
    return StartEnd([this] {IntakeRight.Set(-0.5);}, [this] {IntakeRight.Set(0);});
}

