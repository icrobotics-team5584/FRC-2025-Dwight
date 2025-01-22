#pragma once

#include <iostream>
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
#include <frc/smartdashboard/SmartDashboard.h>

class SwerveIO {
  public:
    virtual void ConfigTurnMotor() = 0;
    virtual void SetDesiredAngle(units::degree_t angle) = 0;
    virtual void SetAngle(units::turn_t angle) = 0;
    virtual void SendSensorsToDash() = 0;
    virtual void SetDesiredVelocity(units::meters_per_second_t velocity, units::newton_t forceFF) = 0;
    virtual void DriveStraightVolts(units::volt_t volts) = 0;
    virtual void StopMotors() = 0;
    virtual void UpdateSim(units::second_t deltaTime) = 0;
    virtual void SetNeutralMode(bool brakeModeToggle) = 0;
    virtual void ConfigDriveMotor() = 0;
    virtual frc::SwerveModulePosition GetPosition() = 0;
    virtual frc::Rotation2d GetAngle() = 0;
    virtual units::meters_per_second_t GetSpeed() = 0;
    virtual units::volt_t GetDriveVoltage() = 0;
    virtual frc::SwerveModuleState GetState() = 0;
    virtual units::radian_t GetDrivenRotations() = 0;
};