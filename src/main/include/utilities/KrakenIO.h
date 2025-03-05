#pragma once

#include <utilities/IOSwerve.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <units/angle.h>
#include <memory>
#include <numbers>
#include <frc/system/plant/LinearSystemId.h>

class KrakenIO : public SwerveIO {
  public:
    KrakenIO(int turnCanID, int driveCanID, int encoderCanID, units::turn_t cancoderMagOffset);
    void ConfigTurnMotor() override;
    void SetDesiredAngle(units::degree_t angle) override;
    void SetAngle(units::turn_t angle) override;
    void SendSensorsToDash() override;
    void SetDesiredVelocity(units::meters_per_second_t velocity, units::newton_t forceFF) override;
    void DriveStraightVolts(units::volt_t volts) override;
    void StopMotors() override;
    void UpdateSim(units::second_t deltaTime) override;
    void SetNeutralMode(bool brakeModeToggle) override;
    void ConfigDriveMotor() override;
    frc::SwerveModulePosition GetPosition() override;
    frc::Rotation2d GetAngle() override;
    units::meters_per_second_t GetSpeed() override;
    units::volt_t GetDriveVoltage() override;
    frc::SwerveModuleState GetState() override;
    units::radian_t GetDrivenRotations() override;

    const double TURNING_GEAR_RATIO = 150.0 / 7.0;
    const double DRIVE_GEAR_RATIO = 6.75;  // L2 - Fast kit
    const units::meter_t WHEEL_RADIUS = 0.0502044712_m;
    const units::meter_t WHEEL_CIRCUMFERENCE = 2 * std::numbers::pi * WHEEL_RADIUS;

    const double TURN_P = 60.0;
    const double TURN_I = 0.0;
    const double TURN_D = 0;
    const double DRIVE_P = 0.017401;  // left
    const double DRIVE_I = 0.0;
    const double DRIVE_D = 0.0;
    const double DRIVE_F = 0;
    const double DRIVE_S = 0.096844;  // Units is V 0.3017; left
    const double DRIVE_V = 0.78528125;
    const double DRIVE_A = 0.079385;  // Units is V/1m/s^2 0.34324; left

  private:

    ctre::phoenix6::hardware::TalonFX _canTurnMotor;
    ctre::phoenix6::configs::TalonFXConfiguration _configTurnMotor{};

    ctre::phoenix6::hardware::TalonFX _canDriveMotor;
    ctre::phoenix6::configs::TalonFXConfiguration _configDriveMotor{};

    frc::DCMotor _driveMotorModel =
        frc::DCMotor::KrakenX60FOC().WithReduction(DRIVE_GEAR_RATIO);
    frc::DCMotor _turnMotorModel =
        frc::DCMotor::Falcon500().WithReduction(TURNING_GEAR_RATIO);

    frc::sim::DCMotorSim _turnMotorSim{
        frc::LinearSystemId::DCMotorSystem(frc::DCMotor::Falcon500(), 0.000000001_kg_sq_m,
                                           TURNING_GEAR_RATIO),
        _turnMotorModel};

    frc::sim::DCMotorSim _driveMotorSim{
        frc::LinearSystemId::DCMotorSystem(frc::DCMotor::KrakenX60FOC(), 0.01_kg_sq_m,
                                           DRIVE_GEAR_RATIO),
        _driveMotorModel};
};
