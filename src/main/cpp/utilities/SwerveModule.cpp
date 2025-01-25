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
                           units::turn_t cancoderMagOffset)
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
  _cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1_tr;
  _cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue::CounterClockwise_Positive;
  _cancoderConfig.MagnetSensor.MagnetOffset = cancoderMagOffset;
  _cancoder.GetConfigurator().Apply(_cancoderConfig);
  frc::SmartDashboard::PutNumber("swerve/cancoder "+std::to_string(canTurnEncoderID) + " mag offset", cancoderMagOffset.value());

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

void SwerveModule::SetDesiredState(
  frc::SwerveModuleState& referenceState,
  units::newton_t xForceFF,
  units::newton_t yForceFF
) {
  // Optimize the reference state to avoid spinning further than 90 degrees
  auto curAngle = GetAngle();
  referenceState.Optimize(curAngle);

  //finds force required for FF, scaled by distance to correct angle error
  namespace math = units::math;
  units::turn_t forceFFAngle    = math::atan(yForceFF / xForceFF);   //finds angle of the force to apply
  units::turn_t angleError      = forceFFAngle - curAngle.Radians(); //checks error between angle to apply force on and current angle
  units::newton_t forceFFNorm   = math::sqrt(math::pow<2>(xForceFF) + math::pow<2>(yForceFF)); //finds required amount of force
  units::newton_t scaledForceFF = forceFFNorm * math::cos(angleError); // scales force by depending on the distance from required angle

  // matches velocity's sign and the FF force
  scaledForceFF = math::copysign(scaledForceFF, referenceState.speed);

  // Slow down drive speed when not pointing the right way. This results in smoother driving.
  referenceState.CosineScale(curAngle);

  // Drive! These functions do some conversions and send targets to falcons
  SetDesiredAngle(referenceState.angle.Degrees());
  SetDesiredVelocity(referenceState.speed, scaledForceFF);
}

frc::SwerveModulePosition SwerveModule::GetPosition() {
  return _io->GetPosition();
}

void SwerveModule::SendSensorsToDash() {
  _io->SendSensorsToDash();
  std::string turnEncoderName = "swerve/turn encoder/" + std::to_string(_cancoder.GetDeviceID());
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

frc::SwerveModuleState SwerveModule::GetCANCoderState() {
  return {GetSpeed(), GetCanCoderAngle()};
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

void SwerveModule::SetDesiredVelocity(units::meters_per_second_t velocity, units::newton_t forceFF) {
  _io->SetDesiredVelocity(velocity, forceFF);
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
}
