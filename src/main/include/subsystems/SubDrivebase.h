// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <AHRS.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/controller/HolonomicDriveController.h>
#include <numbers>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include "Constants.h"
#include "utilities/SwerveModule.h"
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/sysid/SysIdRoutine.h>

class SubDrivebase : public frc2::SubsystemBase {
 public:
  SubDrivebase();
  static SubDrivebase &GetInstance() {
    static SubDrivebase inst;
    return inst;
  }
  void Periodic() override;
  void SimulationPeriodic() override;
  void Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
             units::degrees_per_second_t rot, bool fieldRelative);
  void StopDriving();
  void AddVisionMeasurement(frc::Pose2d pose, double ambiguity, units::second_t timeStamp);
  void ResetGyroHeading(units::degree_t startingAngle = 0_deg);
  void UpdatePosition(frc::Pose2d robotPosition);
  void DriveToPose(frc::Pose2d targetPose);
  void RotateToZero(units::degree_t rotationError);
  void TranslateToZero(units::degree_t translationError);
  void DisplayTrajectory(std::string name, frc::Trajectory trajectory);
  void SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue mode);
  void SetPose(frc::Pose2d pose);
  void DisplayPose(std::string label, frc::Pose2d pose);
  void UpdateOdometry();
  void SyncSensors();
  
  bool IsAtPose(frc::Pose2d pose);
  units::degree_t GetPitch();
  frc::Pose2d GetPose();
  frc::Rotation2d GetHeading();
  units::meters_per_second_t GetVelocity();
  frc::SwerveDriveKinematics<4> GetKinematics();
  frc::ChassisSpeeds GetRobotRelativeSpeeds();
  frc2::CommandPtr WheelCharecterisationCmd();

  static constexpr units::meters_per_second_t MAX_VELOCITY = 6.1_mps;
  static constexpr units::degrees_per_second_t MAX_ANGULAR_VELOCITY = 360_deg_per_s;
  static constexpr units::radians_per_second_squared_t MAX_ANG_ACCEL{std::numbers::pi};

  double MAX_JOYSTICK_ACCEL = 3;
  double MAX_ANGULAR_JOYSTICK_ACCEL = 3;

  // Commands
  frc2::CommandPtr JoystickDrive(frc2::CommandXboxController& controller, bool optionalRotationControl);
  frc2::CommandPtr SyncSensorBut();
  frc2::CommandPtr ResetGyroCmd();
  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction) {
    return _sysIdRoutine.Quasistatic(direction);
  }
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction) {
    return _sysIdRoutine.Dynamic(direction);
  }

 private:
  AHRS _gyro{frc::SerialPort::kMXP};

  // Swerve modules
  frc::Translation2d _frontLeftLocation{160.509_mm, 308.33_mm};
  frc::Translation2d _frontRightLocation{160.509_mm, -308.33_mm};
  frc::Translation2d _backLeftLocation{-306.141_mm, 308.33_mm};
  frc::Translation2d _backRightLocation{-306.141_mm, -308.33_mm};

  const double FRONT_RIGHT_MAG_OFFSET = -0.375732;
  const double FRONT_LEFT_MAG_OFFSET = -0.941406;
  const double BACK_RIGHT_MAG_OFFSET = -0.462891;
  const double BACK_LEFT_MAG_OFFSET = -0.329590;

  SwerveModule _frontLeft{canid::DriveBaseFrontLeftDrive, canid::DriveBaseFrontLeftTurn,
                          canid::DriveBaseFrontLeftEncoder, FRONT_LEFT_MAG_OFFSET};
  SwerveModule _frontRight{canid::DriveBaseFrontRightDrive, canid::DriveBaseFrontRightTurn,
                           canid::DriveBaseFrontRightEncoder, FRONT_RIGHT_MAG_OFFSET};
  SwerveModule _backLeft{canid::DriveBaseBackLeftDrive, canid::DriveBaseBackLeftTurn,
                         canid::DriveBaseBackLeftEncoder, BACK_LEFT_MAG_OFFSET};
  SwerveModule _backRight{canid::DriveBaseBackRightDrive, canid::DriveBaseBackRightTurn,
                          canid::DriveBaseBackRightEncoder, BACK_RIGHT_MAG_OFFSET};

  // Control objects
  frc::SwerveDriveKinematics<4> _kinematics{_frontLeftLocation, _frontRightLocation,
                                            _backLeftLocation, _backRightLocation};

  frc::PIDController Xcontroller{0.2, 0, 0};
  frc::PIDController Ycontroller{0.5, 0, 0};
  frc::ProfiledPIDController<units::radian> Rcontroller{
      6, 0, 0.3, {MAX_ANGULAR_VELOCITY, MAX_ANG_ACCEL}};
  frc::HolonomicDriveController _driveController{Xcontroller, Ycontroller, Rcontroller};

  // Pose estimation
  frc::SwerveDrivePoseEstimator<4> _poseEstimator{
      _kinematics,
      _gyro.GetRotation2d(),
      {frc::SwerveModulePosition{0_m, _frontLeft.GetAngle()},
       frc::SwerveModulePosition{0_m, _frontRight.GetAngle()},
       frc::SwerveModulePosition{0_m, _backLeft.GetAngle()},
       frc::SwerveModulePosition{0_m, _backRight.GetAngle()}},
      frc::Pose2d()};

  frc::Field2d _fieldDisplay;
  frc::Pose2d _prevPose;  // Used for velocity calculations

  // Drive requests
  units::meters_per_second_t _forwardSpeedRequest = 0_mps;
  units::meters_per_second_t _sidewaysSpeedRequest = 0_mps;
  units::degrees_per_second_t _rotationSpeedRequest = 0_deg_per_s;
  bool _fieldOrientedRequest = true; 

  // Sysid
  frc2::sysid::SysIdRoutine _sysIdRoutine{
      frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt, std::nullopt},
      frc2::sysid::Mechanism{[this](units::volt_t driveVoltage) {
                               _frontLeft.DriveStraightVolts(driveVoltage);
                               _backLeft.DriveStraightVolts(driveVoltage);
                               _frontRight.DriveStraightVolts(driveVoltage);
                               _backRight.DriveStraightVolts(driveVoltage);
                             },
                             [this](frc::sysid::SysIdRoutineLog* log) {
                               log->Motor("drive-left")
                                   .voltage(_frontLeft.GetDriveVoltage())
                                   .position(_frontLeft.GetDrivenRotations().convert<units::turns>())
                                   .velocity(_frontLeft.GetSpeed());
                               log->Motor("drive-right")
                                   .voltage(_frontRight.GetDriveVoltage())
                                   .position(_frontRight.GetDrivenRotations().convert<units::turns>())
                                   .velocity(_frontRight.GetSpeed());
                             },
                             this}};
};