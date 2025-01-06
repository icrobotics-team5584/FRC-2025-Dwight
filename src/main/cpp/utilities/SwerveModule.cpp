// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utilities/SwerveModule.h"
// #include "utilities/Conversion.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>
#include <iostream>
#include <utilities/IOSwerve.h>
#include "utilities/NeoIO.h"
#include "utilities/KrakenIO.h"
#include "utilities/BotVars.h"

SwerveModule::SwerveModule(int canDriveMotorID, int canTurnMotorID, int canTurnEncoderID,
                           double cancoderMagOffset)
    : _cancoder(canTurnEncoderID){
  using namespace ctre::phoenix6::signals;
  using namespace ctre::phoenix6::configs;

  // Select IO interface
  if (BotVars::GetRobot() == BotVars::PRACTICE){
    //CHANGE TO NEOIO
    _io = std::make_unique<NeoIO>(canTurnMotorID, canDriveMotorID, canTurnEncoderID, cancoderMagOffset);
  }
  else {
    _io = std::make_unique<KrakenIO>(canTurnMotorID, canDriveMotorID, canTurnEncoderID, cancoderMagOffset);
  }

  // Config CANCoder 
  _cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue::Unsigned_0To1;
  _cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue::CounterClockwise_Positive;
  _cancoderConfig.MagnetSensor.MagnetOffset = cancoderMagOffset;
  _cancoder.GetConfigurator().Apply(_cancoderConfig);
  frc::SmartDashboard::PutNumber("swerve/cancoder "+std::to_string(canTurnEncoderID) + " mag offset", cancoderMagOffset);

  // Config Motors, dont change order of function calls. Or configs will not be applied before sensors are synced
  ConfigTurnMotor();
  ConfigDriveMotor();
  SyncSensors();
}

void SwerveModule::ConfigTurnMotor(){
  _io->ConfigTurnMotor();
}

void SwerveModule::ConfigDriveMotor(){
  _io->ConfigDriveMotor();
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  auto targetState = frc::SwerveModuleState::Optimize(referenceState, GetAngle());

  // Drive! These functions do some conversions and send targets to falcons
  SetDesiredAngle(targetState.angle.Degrees());
  SetDesiredVelocity(targetState.speed);
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return _io->GetPosition();
}

void SwerveModule::SendSensorsToDash() {
  _io->SendSensorsToDash();
  std::string turnEncoderName = "swerve/turn encoder/" + std::to_string(_cancoderID);
  frc::SmartDashboard::PutNumber(turnEncoderName + "Abs position", _cancoder.GetAbsolutePosition().GetValue().value());
  frc::SmartDashboard::PutNumber(turnEncoderName + "Supply Voltage", _cancoder.GetSupplyVoltage().GetValue().value());
  frc::SmartDashboard::PutString(turnEncoderName + "Magnet Health", _cancoder.GetMagnetHealth().GetValue().ToString());
  frc::SmartDashboard::PutNumber(turnEncoderName + "Velocity", _cancoder.GetVelocity().GetValue().value());
}

frc::Rotation2d SwerveModule::GetAngle() {
  return _io->GetAngle();
}

frc::Rotation2d SwerveModule::GetCanCoderAngle() {
  units::radian_t angle = _cancoder.GetAbsolutePosition().GetValue();
  return angle;
}

units::meters_per_second_t SwerveModule::GetSpeed() {
  return _io->GetSpeed();
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {GetSpeed(), GetAngle()};
}

units::volt_t SwerveModule::GetDriveVoltage() {
  return _io->GetDriveVoltage();
}

units::radian_t SwerveModule::GetDrivenRotations() {
  return _io->GetDrivenRotations();
}

void SwerveModule::SetDesiredAngle(units::degree_t angle) {
  _io->SetDesiredAngle(angle);
}

void SwerveModule::SetDesiredVelocity(units::meters_per_second_t velocity) {
  _io->SetDesiredVelocity(velocity);
}

void SwerveModule::DriveStraightVolts(units::volt_t volts) {
  _io->DriveStraightVolts(volts);
}

void SwerveModule::StopMotors() {
  _io->StopMotors();
}

void SwerveModule::SetBreakMode(bool enableBreakMode) {
  _io->SetNeutralMode(enableBreakMode);
}

void SwerveModule::SyncSensors() {
  _io->SetAngle(_cancoder.GetAbsolutePosition().GetValue());
}

void SwerveModule::UpdateSim(units::second_t deltaTime) {
  _io->UpdateSim(deltaTime);
  // CANcoders are attached directly to the mechanism, so don't account for the steer gearing
  auto& cancoderState = _cancoder.GetSimState();
  cancoderState.SetRawPosition(_turnMotorSim.GetAngularPosition());
  cancoderState.SetVelocity(_turnMotorSim.GetAngularVelocity());
}
