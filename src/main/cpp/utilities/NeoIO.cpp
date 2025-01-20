#include "utilities/NeoIO.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <string>
#include <rev/SparkMax.h>

NeoIO::NeoIO(int turnCanID, int driveCanID, int encoderCanID,
             units::turn_t cancoderMagOffset) : _canTurnMotor(turnCanID, 40_A), _canDriveMotor(driveCanID, 40_A), _canEncoder(encoderCanID)
{
    frc::SmartDashboard::PutData("Swerve/DriveMotor"+ std::to_string(driveCanID), (wpi::Sendable*) &_canDriveMotor);
    frc::SmartDashboard::PutData("Swerve/TurnMotor" + std::to_string(turnCanID), (wpi::Sendable*) &_canTurnMotor);
}

void NeoIO::ConfigTurnMotor(){
    rev::spark::SparkBaseConfig _canTurnConfig;

    _canTurnConfig.encoder.PositionConversionFactor(1.0 / TURNING_GEAR_RATIO).VelocityConversionFactor(TURNING_GEAR_RATIO / 60);
    _canTurnConfig.closedLoop.Pid(TURN_P, TURN_I, TURN_D);
    _canTurnConfig.closedLoop.PositionWrappingEnabled(true)
    .PositionWrappingMinInput(0)
    .PositionWrappingMaxInput(1);
    _canTurnConfig.Inverted(true)
    .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);

    _canTurnMotor.AdjustConfig(_canTurnConfig);
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

void NeoIO::SetDesiredVelocity(units::meters_per_second_t velocity, units::newton_t forceFF){
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
    rev::spark::SparkBaseConfig _neutralModeConfig;

    if (brakeModeToggle == true){
        _neutralModeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
        _canDriveMotor.AdjustConfig(_neutralModeConfig);
        _canTurnMotor.AdjustConfig(_neutralModeConfig);
    }
    else if (brakeModeToggle == false){
        _neutralModeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
        _canDriveMotor.AdjustConfig(_neutralModeConfig);        
        _canTurnMotor.AdjustConfig(_neutralModeConfig);
    }
}

void NeoIO::ConfigDriveMotor(){
    rev::spark::SparkBaseConfig _canDriveConfig;

    _canDriveConfig.SmartCurrentLimit(40);
    _canDriveConfig.closedLoop.Pidf(DRIVE_P, DRIVE_I, DRIVE_D, DRIVE_FF);
    _canDriveConfig.encoder.PositionConversionFactor(1.0 / DRIVE_GEAR_RATIO)
    .VelocityConversionFactor(DRIVE_GEAR_RATIO / 60);
    _canDriveConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);

    _canDriveMotor.AdjustConfig(_canDriveConfig);
}

frc::SwerveModulePosition NeoIO::GetPosition(){
 units::meter_t distance = _canDriveMotor.GetPosition()*WHEEL_CIRCUMFERENCE;
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