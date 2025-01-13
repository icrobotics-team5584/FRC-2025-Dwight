// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubIntake.h"
#include <rev/SparkMax.h>
#include <ctre/phoenix6/controls/PositionTorqueCurrentFOC.hpp>
#include <frc/smartdashboard/SmartDashboard.h>
#include "utilities/RobotLogs.h"

SubIntake::SubIntake() {
    _configIntakePivotMotor.Feedback.SensorToMechanismRatio = ARM_GEARING;
    _configIntakePivotMotor.Slot0.kP = P;
    _configIntakePivotMotor.Slot0.kI = I;
    _configIntakePivotMotor.Slot0.kD = D;
    _configIntakePivotMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    _configIntakePivotMotor.CurrentLimits.StatorCurrentLimitEnable = true;
    _configIntakePivotMotor.CurrentLimits.StatorCurrentLimit = 50_A;
    _configIntakePivotMotor.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    _configIntakePivotMotor.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    _configIntakePivotMotor.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.25_tr;
    _configIntakePivotMotor.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0_tr;
    _intakePivotMotor.GetConfigurator().Apply(_configIntakePivotMotor);
    frc::SmartDashboard::PutData("Intake/armMechDisplay", &_singleJointedArmMech);
    frc::SmartDashboard::PutData("Intake/rollerMechDisplay", &_rollerMech);
}

// This method will be called once per scheduler run
void SubIntake::Periodic() {
    units::angle::degree_t pivotAngle = _intakePivotMotor.GetPosition().GetValue();
    _arm1Ligament->SetAngle(pivotAngle);
    Logger::LogFalcon("Intake/pivotMotor", _intakePivotMotor);

    units::angle::degree_t rollerAngle = _intakeMotor.GetPosition()/10;
    frc::SmartDashboard::PutNumber("rollerAngle", rollerAngle.value());
    _rollerMechLeft.SetAngle(rollerAngle);
    _rollerMechRight.SetAngle(rollerAngle*-1);
}

void SubIntake::SimulationPeriodic() {
    auto& motorState = _intakePivotMotor.GetSimState();
    _intakeSim.SetInputVoltage(motorState.GetMotorVoltage());
    _intakeSim.Update(20_ms);

    auto armAngle = _intakeSim.GetAngle();
    auto armVel = _intakeSim.GetVelocity();
    motorState.SetRotorVelocity(armVel * ARM_GEARING);
    motorState.SetRawRotorPosition(armAngle * ARM_GEARING);
    frc::SmartDashboard::PutNumber("Intake/armAngle", armAngle.value());
    frc::SmartDashboard::PutNumber("Intake/armVelocity", armVel.value());

    _rollerSim.SetInputVoltage(_intakeMotor.CalcSimVoltage());
    _rollerSim.Update(20_ms);
    _intakeMotor.IterateSim(_rollerSim.GetAngularVelocity());
}   

frc2::CommandPtr SubIntake::Intake() {
    return StartEnd([this] {_intakeMotor.Set(0.5);}, [this] {_intakeMotor.Set(0);});
}

frc2::CommandPtr SubIntake::Outtake() {
    return StartEnd([this] {_intakeMotor.Set(-0.5);}, [this] {_intakeMotor.Set(0);});
}

void SubIntake::SetDesiredAngle(units::degree_t angle){
    if( (angle >= INTAKE_MIN_ANGLE) && (angle <= INTAKE_MAX_ANGLE) ) {
        _intakePivotMotor.SetControl(ctre::phoenix6::controls::PositionVoltage(angle).WithEnableFOC(false));
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

