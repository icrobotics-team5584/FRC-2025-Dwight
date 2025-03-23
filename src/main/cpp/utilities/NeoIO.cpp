#include "utilities/NeoIO.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <string>
#include <rev/SparkMax.h>

NeoIO::NeoIO(int turnCanID, int driveCanID, int encoderCanID,
             units::turn_t cancoderMagOffset)
    : _canTurnMotor(turnCanID), _canDriveMotor(driveCanID), _canEncoder(encoderCanID) {
  frc::SmartDashboard::PutData("Swerve/DriveMotor" + std::to_string(driveCanID), (wpi::Sendable*) &_canDriveMotor);
  frc::SmartDashboard::PutData("Swerve/TurnMotor" + std::to_string(turnCanID), (wpi::Sendable*) &_canTurnMotor);
}

void NeoIO::ConfigTurnMotor() {
  ICSparkConfig turnMotorConfig;

  turnMotorConfig.encoder.positionConversionFactor = 1.0 / TURNING_GEAR_RATIO;
  turnMotorConfig.encoder.velocityConversionFactor = TURNING_GEAR_RATIO / 60.0;
  turnMotorConfig.closedLoop.slots[0].p = TURN_P;
  turnMotorConfig.closedLoop.slots[0].i = TURN_I;
  turnMotorConfig.closedLoop.slots[0].d = TURN_D;
  turnMotorConfig.closedLoop.positionWrappingEnabled = true;
  turnMotorConfig.closedLoop.positionWrappingMinInput = 0_tr;
  turnMotorConfig.closedLoop.positionWrappingMaxInput = 1_tr;
  turnMotorConfig.inverted = true;
  turnMotorConfig.idleMode = rev::spark::SparkBaseConfig::IdleMode::kBrake;
  turnMotorConfig.smartCurrentStallLimit = 40_A;

  _canTurnMotor.OverwriteConfig(turnMotorConfig);
}

void NeoIO::SetDesiredAngle(units::degree_t angle) {
  _canTurnMotor.SetPositionTarget(angle);
}

void NeoIO::SetAngle(units::turn_t angle) {
  /*
    @param angle: angle of encoder
  */
  _canTurnMotor.SetCANTimeout(500);
  int maxAttempts = 15;
  int currentAttempts = 0;
  units::turn_t tolerance = 0.01_tr;
  while (units::math::abs(_canTurnMotor.GetPosition() - angle) > tolerance &&
         currentAttempts < maxAttempts) {
    _canTurnMotor.SetPosition(angle);
    currentAttempts++;
  }

  currentAttempts = 0;
  _canTurnMotor.SetCANTimeout(10);
}

void NeoIO::SendSensorsToDash() {
  // use Brayden logging tool once imported
}

void NeoIO::SetDesiredVelocity(units::meters_per_second_t velocity, units::newton_t forceFF) {
  units::turns_per_second_t TurnsPerSec = (velocity.value() / WHEEL_CIRCUMFERENCE.value()) * 1_tps;
  _canDriveMotor.SetVelocityTarget(TurnsPerSec);
}

void NeoIO::DriveStraightVolts(units::volt_t volts) {
  SetDesiredAngle(0_deg);
  _canDriveMotor.SetVoltage(volts);
}

void NeoIO::StopMotors() {
  _canDriveMotor.Set(0);
  _canTurnMotor.Set(0);
}

void NeoIO::UpdateSim(units::second_t deltaTime) {

}

void NeoIO::SetNeutralMode(bool brakeModeToggle) {
  ICSparkConfig _neutralModeConfig;

  if (brakeModeToggle == true) {
    _neutralModeConfig.idleMode = rev::spark::SparkBaseConfig::IdleMode::kBrake;
    _canDriveMotor.AdjustConfigNoPersist(_neutralModeConfig);
    _canTurnMotor.AdjustConfigNoPersist(_neutralModeConfig);
  } else if (brakeModeToggle == false) {
    _neutralModeConfig.idleMode = rev::spark::SparkBaseConfig::IdleMode::kCoast;
    _canDriveMotor.AdjustConfigNoPersist(_neutralModeConfig);
    _canTurnMotor.AdjustConfigNoPersist(_neutralModeConfig);
  }
}

void NeoIO::ConfigDriveMotor() {
  ICSparkConfig driveMotorConfig;

  driveMotorConfig.smartCurrentStallLimit = 70_A;
  driveMotorConfig.closedLoop.slots[0].p = DRIVE_P;
  driveMotorConfig.closedLoop.slots[0].i = DRIVE_I;
  driveMotorConfig.closedLoop.slots[0].d = DRIVE_D;
  driveMotorConfig.encoder.positionConversionFactor = 1.0 / DRIVE_GEAR_RATIO;
  driveMotorConfig.encoder.velocityConversionFactor = DRIVE_GEAR_RATIO / 60.0;
  driveMotorConfig.idleMode = rev::spark::SparkBaseConfig::IdleMode::kBrake;

  _canDriveMotor.OverwriteConfig(driveMotorConfig);

}

frc::SwerveModulePosition NeoIO::GetPosition() {
  units::meter_t distance = _canDriveMotor.GetPosition().value() * WHEEL_CIRCUMFERENCE;
  return {distance, GetAngle()};
}

frc::Rotation2d NeoIO::GetAngle() {
  units::radian_t turnAngle = _canTurnMotor.GetPosition();
  return turnAngle;
}

units::meters_per_second_t NeoIO::GetSpeed() {
  frc::SmartDashboard::PutNumber("Swerve/module" + std::to_string(_canDriveMotor.GetDeviceId()) + " tps", _canDriveMotor.GetVelocity().value()); 
  frc::SmartDashboard::PutNumber("Swerve/module" + std::to_string(_canDriveMotor.GetDeviceId()) + " mps", (_canDriveMotor.GetVelocity().value() * WHEEL_CIRCUMFERENCE.value())
  ); 
  return (_canDriveMotor.GetVelocity().value() * WHEEL_CIRCUMFERENCE.value()) * 1_mps;
}

units::volt_t NeoIO::GetDriveVoltage() {
  return _canDriveMotor.GetAppliedOutput() * _canDriveMotor.GetBusVoltage() * 1_V;
}

frc::SwerveModuleState NeoIO::GetState() {
  return {GetSpeed(), GetAngle()};
}

units::radian_t NeoIO::GetDrivenRotations() {
  return _canDriveMotor.GetPosition();
}
