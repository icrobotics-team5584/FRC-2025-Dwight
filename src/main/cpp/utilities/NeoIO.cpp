#include "utilities/NeoIO.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <string>

NeoIO::NeoIO(int turnCanID, int driveCanID, int encoderCanID,
             double cancoderMagOffset) : _canTurnMotor(turnCanID), _canDriveMotor(driveCanID), _canEncoder(encoderCanID)
{
    frc::SmartDashboard::PutData("Swerve/DriveMotor"+ std::to_string(driveCanID), (wpi::Sendable*) &_canDriveMotor);
    frc::SmartDashboard::PutData("Swerve/TurnMotor" + std::to_string(turnCanID), (wpi::Sendable*) &_canTurnMotor);
}

void NeoIO::ConfigTurnMotor(){
  _canTurnMotor.SetConversionFactor(1.0 / TURNING_GEAR_RATIO);
  _canTurnMotor.EnableClosedLoopWrapping(0_tr, 1_tr);
  _canTurnMotor.SetPIDFF(TURN_P, TURN_I, TURN_D);
  _canTurnMotor.SetInverted(true);
  _canTurnMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
}

void NeoIO::SetDesiredAngle(units::degree_t angle){
    _canTurnMotor.SetPositionTarget(angle);
}


void NeoIO::SetAngle(units::turn_t angle){
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

void NeoIO::SendSensorsToDash(){
    //use Brayden logging tool once imported
}

void NeoIO::SetDesiredVelocity(units::meters_per_second_t velocity){
    units::turns_per_second_t TurnsPerSec = (velocity.value() / WHEEL_CIRCUMFERENCE.value()) * 1_tps;
    _canDriveMotor.SetVelocityTarget(TurnsPerSec);
}

void NeoIO::DriveStraightVolts(units::volt_t volts){
    SetDesiredAngle(0_deg);
    _canDriveMotor.SetVoltage(volts);
}

void NeoIO::StopMotors(){
    _canDriveMotor.Set(0);
    _canTurnMotor.Set(0);
}

void NeoIO::UpdateSim(units::second_t deltaTime){

}

void NeoIO::SetNeutralMode(bool brakeModeToggle){
    if (brakeModeToggle == true){
        _canDriveMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
        _canTurnMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
    }
    else if (brakeModeToggle == false){
        _canDriveMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
        _canTurnMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kCoast);
    }
}

void NeoIO::ConfigDriveMotor(){
    _canDriveMotor.SetSmartCurrentLimit(40);
    _canDriveMotor.SetP(DRIVE_P);
    _canDriveMotor.SetI(DRIVE_I);
    _canDriveMotor.SetD(DRIVE_D);
    _canDriveMotor.SetFF(DRIVE_FF);
    _canDriveMotor.SetConversionFactor(1.0/DRIVE_GEAR_RATIO);
    _canDriveMotor.SetIdleMode(rev::CANSparkBase::IdleMode::kBrake);
}

frc::SwerveModulePosition NeoIO::GetPosition(){
 units::meter_t distance = 1_m;
 return {distance, GetAngle()};
}

frc::Rotation2d NeoIO::GetAngle(){
    units::radian_t turnAngle = _canTurnMotor.GetPosition();
    return turnAngle;
}

units::meters_per_second_t NeoIO::GetSpeed(){
    return (_canDriveMotor.GetVelocity().value() * WHEEL_CIRCUMFERENCE.value()) * 1_mps;
}

units::volt_t NeoIO::GetDriveVoltage(){
    return _canDriveMotor.GetAppliedOutput() * _canDriveMotor.GetBusVoltage() * 1_V;
}

frc::SwerveModuleState NeoIO::GetState(){
    return {GetSpeed(), GetAngle()};
}

units::radian_t NeoIO::GetDrivenRotations(){
    return _canDriveMotor.GetPosition();
}