#include <utilities/KrakenIO.h>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <units/math.h>
#include <units/angle.h>
#include <ctre/phoenix6/controls/PositionTorqueCurrentFOC.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <frc/smartdashboard/SmartDashboard.h>

const bool FOCstate = true;

KrakenIO::KrakenIO(int turnCanID, int driveCanID, int encoderCanID,
                   units::turn_t cancoderMagOffset) : _canTurnMotor(turnCanID),
                                               _canDriveMotor(driveCanID)
{
}

void KrakenIO::ConfigTurnMotor(){
    _configTurnMotor.Feedback.SensorToMechanismRatio = TURNING_GEAR_RATIO;
    _configTurnMotor.ClosedLoopGeneral.ContinuousWrap = true;
    _configTurnMotor.Slot0.kP = TURN_P;
    _configTurnMotor.Slot0.kI = TURN_I;
    _configTurnMotor.Slot0.kD = TURN_D;
    _configTurnMotor.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    _configTurnMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
    _canTurnMotor.GetConfigurator().Apply(_configTurnMotor);
    }


void KrakenIO::SetDesiredAngle(units::degree_t angle){
  _canTurnMotor.SetControl(ctre::phoenix6::controls::PositionVoltage(angle).WithEnableFOC(FOCstate));
}


void KrakenIO::SetAngle(units::turn_t angle) {
  _canTurnMotor.SetPosition(angle);
}


void KrakenIO::SendSensorsToDash(){
  std::string driveMotorName = "swerve/drive motor/" + std::to_string(_canDriveMotor.GetDeviceID());
  std::string turnMotorName = "swerve/turn motor/" + std::to_string(_canTurnMotor.GetDeviceID());

  frc::SmartDashboard::PutNumber(driveMotorName + "Target velocity", _canDriveMotor.GetClosedLoopReference().GetValue());
  frc::SmartDashboard::PutNumber(driveMotorName + "velocity", _canDriveMotor.GetVelocity().GetValue().value());
  frc::SmartDashboard::PutNumber(turnMotorName  + "position", _canTurnMotor.GetPosition().GetValue().value());
  frc::SmartDashboard::PutNumber(turnMotorName  + "voltage", _canTurnMotor.GetMotorVoltage().GetValueAsDouble());
  frc::SmartDashboard::PutNumber(turnMotorName  + "target", _canTurnMotor.GetClosedLoopReference().GetValue());
  frc::SmartDashboard::PutNumber(turnMotorName  + "error", _canTurnMotor.GetClosedLoopError().GetValue());
}

void KrakenIO::SetDesiredVelocity(units::meters_per_second_t velocity){
  units::turns_per_second_t TurnsPerSec = (velocity.value() / WHEEL_CIRCUMFERENCE.value()) * 1_tps;

  _canDriveMotor.SetControl(ctre::phoenix6::controls::VelocityVoltage{(TurnsPerSec)});
}

void KrakenIO::DriveStraightVolts(units::volt_t volts){
  SetDesiredAngle(0_deg);
  _canDriveMotor.SetControl(ctre::phoenix6::controls::VoltageOut{volts});
}

void KrakenIO::StopMotors(){
  _canDriveMotor.Set(0);
  _canTurnMotor.Set(0);
}

void KrakenIO::SetNeutralMode(bool brakeModeToggle){
  if (brakeModeToggle == true){
    _configTurnMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  }
  else if (brakeModeToggle == false){
    _configDriveMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
  }

  _canTurnMotor.GetConfigurator().Apply(_configTurnMotor);
  _canDriveMotor.GetConfigurator().Apply(_configDriveMotor);
}

void KrakenIO::ConfigDriveMotor(){
  _configDriveMotor.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RotorSensor;
  _configDriveMotor.ClosedLoopGeneral.ContinuousWrap = false;
  _configDriveMotor.Feedback.SensorToMechanismRatio = DRIVE_GEAR_RATIO;
  _configDriveMotor.Slot0.kP = DRIVE_P;
  _configDriveMotor.Slot0.kI = DRIVE_I;
  _configDriveMotor.Slot0.kD = DRIVE_D;
  _configDriveMotor.CurrentLimits.SupplyCurrentLimitEnable = true;
  _configDriveMotor.CurrentLimits.StatorCurrentLimitEnable = true;
  _configDriveMotor.CurrentLimits.SupplyCurrentLowerLimit = 40.0_A;
  _configDriveMotor.CurrentLimits.SupplyCurrentLimit = 60.0_A;
  _configDriveMotor.CurrentLimits.SupplyCurrentLowerTime = 0.1_s;
  _configDriveMotor.CurrentLimits.StatorCurrentLimit = 80.0_A; //Untested
  _configDriveMotor.Slot0.kS = DRIVE_S;
  _configDriveMotor.Slot0.kV = DRIVE_V;
  _configDriveMotor.Slot0.kA = DRIVE_A;
  _configDriveMotor.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
  _canDriveMotor.GetConfigurator().Apply(_configDriveMotor);
}

frc::SwerveModulePosition KrakenIO::GetPosition(){
    auto wheelRot = ctre::phoenix6::BaseStatusSignal::GetLatencyCompensatedValue(
      _canDriveMotor.GetPosition(), _canDriveMotor.GetVelocity());
  units::meter_t distance = (WHEEL_CIRCUMFERENCE.value() * wheelRot.value()) * 1_m;
  return {distance, GetAngle()};
}

frc::Rotation2d KrakenIO::GetAngle(){
    units::radian_t turnAngle = _canTurnMotor.GetPosition().GetValue();
  return turnAngle;
}

units::meters_per_second_t KrakenIO::GetSpeed(){
    return (_canDriveMotor.GetVelocity().GetValue().value() * WHEEL_CIRCUMFERENCE.value()) * 1_mps;
}

units::volt_t KrakenIO::GetDriveVoltage(){
  return _canDriveMotor.GetMotorVoltage().GetValue();
}

frc::SwerveModuleState KrakenIO::GetState(){
    return {GetSpeed(), GetAngle()};
}

units::radian_t KrakenIO::GetDrivenRotations(){
    return _canDriveMotor.GetPosition().GetValue();
}

void KrakenIO::UpdateSim(units::second_t deltaTime){
  // Drive motor
  auto& driveState = _canDriveMotor.GetSimState();
  _driveMotorSim.SetInputVoltage(driveState.GetMotorVoltage());
  _driveMotorSim.Update(deltaTime);
  driveState.SetRawRotorPosition(_driveMotorSim.GetAngularPosition() * DRIVE_GEAR_RATIO);
  driveState.SetRotorVelocity(_driveMotorSim.GetAngularVelocity() * DRIVE_GEAR_RATIO);

  // Turn Motor
  auto& turnState = _canTurnMotor.GetSimState();
  _turnMotorSim.SetInputVoltage(turnState.GetMotorVoltage());
  _turnMotorSim.Update(deltaTime);
  turnState.SetRawRotorPosition(_turnMotorSim.GetAngularPosition() * TURNING_GEAR_RATIO);
  turnState.SetRotorVelocity(_turnMotorSim.GetAngularVelocity() * TURNING_GEAR_RATIO);
}