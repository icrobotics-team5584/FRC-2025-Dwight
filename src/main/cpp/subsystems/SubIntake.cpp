// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIntake.h"
#include <rev/SparkMax.h>
#include <ctre/phoenix6/controls/PositionTorqueCurrentFOC.hpp>
#include <frc/smartdashboard/SmartDashboard.h>
#include "utilities/RobotLogs.h"


SubIntake::SubIntake() {
    frc::SmartDashboard::PutData("Intake/Motor", &_intakePivotMotor);
    rev::spark::SparkBaseConfig _intakePivotMotorConfig;

    _intakePivotMotorConfig.encoder.PositionConversionFactor(1/ARM_GEARING); // change to just ARM_GEARING instead of 1/ARM_GEARING for simulation
    _intakePivotMotorConfig.encoder.VelocityConversionFactor(ARM_GEARING/60.0);
    _intakePivotMotorConfig.closedLoop.Pid(P, I, D, rev::spark::ClosedLoopSlot::kSlot0);
    // _intakePivotMotorConfig.closedLoop.PositionWrappingEnabled(true)
    //     .PositionWrappingMinInput(-10000000)
    //     .PositionWrappingMaxInput(10000000);
    _intakePivotMotorConfig.Inverted(false)
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    _intakePivotMotor.AdjustConfig(_intakePivotMotorConfig);

    frc::SmartDashboard::PutData("Intake/armMechDisplay", &_singleJointedArmMech);
    frc::SmartDashboard::PutData("Intake/rollerMechDisplay", &_rollerMech);
}

// This method will be called once per scheduler run
void SubIntake::Periodic() {
    frc::SmartDashboard::PutNumber("Intake/MotorTarget", _intakePivotMotor.GetPositionTarget().value());
    frc::SmartDashboard::PutData("Intake/armMechDisplay", &_singleJointedArmMech);
    frc::SmartDashboard::PutNumber("Intake/MotorVoltage", _intakePivotMotor.GetMotorVoltage().value());
    frc::SmartDashboard::PutNumber("Intake/Current", _intakePivotMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Intake/PositionMotor", (_intakePivotMotor.GetPosition().value()));
    frc::SmartDashboard::PutNumber("Intake/PositionArm", ((_intakePivotMotor.GetPosition().value())/ARM_GEARING));
}

void SubIntake::SimulationPeriodic() {
    units::angle::degree_t pivotAngle = _intakePivotMotor.GetPosition();
    _arm1Ligament->SetAngle(pivotAngle);
    _intakeSim.SetInputVoltage(_intakePivotMotor.CalcSimVoltage());
    _intakeSim.Update(20_ms);

    auto armAngle = _intakeSim.GetAngle();
    auto armVel = _intakeSim.GetVelocity();
    _intakePivotMotor.IterateSim(armVel);

    frc::SmartDashboard::PutNumber("Intake/armAngle", armAngle.value());
    frc::SmartDashboard::PutNumber("Intake/armVelocity", armVel.value());
    frc::SmartDashboard::PutNumber("Intake/SimVoltage", _intakePivotMotor.CalcSimVoltage().value());
}   

frc2::CommandPtr SubIntake::Intake() {
    return StartEnd([this] {_intakeMotor.Set(0.5);}, [this] {_intakeMotor.Set(0);});
}

frc2::CommandPtr SubIntake::Outtake() {
    return StartEnd([this] {_intakeMotor.Set(-0.5);}, [this] {_intakeMotor.Set(0);});
}

void SubIntake::SetDesiredAngle(units::degree_t angle){
    if( (angle >= INTAKE_MIN_ANGLE) && (angle <= INTAKE_MAX_ANGLE) ) {
        _intakePivotMotor.SetPositionTarget(angle.value()*1_tr);
    } 
}

frc2::CommandPtr SubIntake::Deploy() {
    return RunOnce([this] {SetDesiredAngle(0_deg);});
}

frc2::CommandPtr SubIntake::DeployAndRetract() {
    return StartEnd([this] {SubIntake::Deploy();}, [this] {SubIntake::Stow();});
}

frc2::CommandPtr SubIntake::Stow() {
    return RunOnce([this] {SetDesiredAngle(90_deg);});
}



