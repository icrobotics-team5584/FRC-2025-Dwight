#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <utilities/ICSparkMax.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/simulation/DCMotorSim.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/length.h>
#include <units/angular_velocity.h>
#include <memory>
#include <numbers>
#include <utilities/IOSwerve.h>
#include <frc/system/plant/LinearSystemId.h>


class SwerveModule {
 public:
  SwerveModule(int canDriveMotorID, int canTurnMotorID, int canTurnEncoderID, units::turn_t cancoderMagOffset); 
  void SetDesiredState(frc::SwerveModuleState& state, 
    units::newton_t xForceFF = 0_N,
    units::newton_t yForceFF = 0_N); 
  void SyncSensors();
  void SendSensorsToDash();
  void SetDesiredAngle(units::degree_t angle);
  void SetDesiredVelocity(units::meters_per_second_t velocity, units::newton_t forceFF);
  void DriveStraightVolts(units::volt_t volts);
  void StopMotors();
  void UpdateSim(units::second_t deltaTime);
  void SetBreakMode(bool enableBreakMode);
  void ConfigTurnMotor();
  void ConfigDriveMotor();
  frc::SwerveModulePosition GetPosition();
  frc::Rotation2d GetAngle();
  frc::Rotation2d GetCanCoderAngle();
  units::meters_per_second_t GetSpeed();
  units::volt_t GetDriveVoltage();
  frc::SwerveModuleState GetState();
  frc::SwerveModuleState GetCANCoderState();
  units::radian_t GetDrivenRotations();

 private:
  std::unique_ptr<SwerveIO> _io;

  ctre::phoenix6::hardware::CANcoder _cancoder;
  ctre::phoenix6::configs::CANcoderConfiguration _cancoderConfig;


};
