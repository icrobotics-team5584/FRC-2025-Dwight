// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIntake.h"
#include <rev/SparkMax.h>
#include <ctre/phoenix6/controls/PositionTorqueCurrentFOC.hpp>

SubIntake::SubIntake() {
    rev::spark::SparkBaseConfig intakeLeftConfig;
    intakeLeftConfig.Follow(_intakeRightMotor.GetDeviceId(), true);
    _intakeLeftMotor.AdjustConfig(intakeLeftConfig);
    _configIntakePivotMotor.Feedback.SensorToMechanismRatio = GearRatio;
    _configIntakePivotMotor.Slot0.kP = P;
    _configIntakePivotMotor.Slot0.kI = I;
    _configIntakePivotMotor.Slot0.kD = D;
    _configIntakePivotMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    _intakePivotMotor.GetConfigurator().Apply(_configIntakePivotMotor);
  
}

// This method will be called once per scheduler run
void SubIntake::Periodic() {}

frc2::CommandPtr SubIntake::Intake() {
    return StartEnd([this] {_intakeRightMotor.Set(0.5);}, [this] {_intakeRightMotor.Set(0);});
}

frc2::CommandPtr SubIntake::Outtake() {
    return StartEnd([this] {_intakeRightMotor.Set(-0.5);}, [this] {_intakeRightMotor.Set(0);});
}

void SubIntake::SetDesiredAngle(units::degree_t angle){
    _intakePivotMotor.SetControl(ctre::phoenix6::controls::PositionVoltage(angle).WithEnableFOC(true));
}

frc2::CommandPtr SubIntake::Deploy() {
    return RunOnce([this] {SetDesiredAngle(90_deg);});
}

frc2::CommandPtr SubIntake::Stow() {
    return RunOnce([this] {SetDesiredAngle(0_deg);});
}

