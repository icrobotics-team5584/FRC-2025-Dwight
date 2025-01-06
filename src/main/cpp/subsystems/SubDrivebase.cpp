// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>
#include <units/time.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>
#include "subsystems/SubDrivebase.h"
#include <frc/filter/SlewRateLimiter.h>
#include <iostream>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>



SubDrivebase::SubDrivebase() {
  frc::SmartDashboard::PutNumber("Drivebase/Config/MaxVelocity", MAX_VELOCITY.value());
  frc::SmartDashboard::PutNumber("Drivebase/Config/MaxAngularVelocity",
                                 MAX_ANGULAR_VELOCITY.value());
  frc::SmartDashboard::PutNumber("Drivebase/Config/MaxAcceleration", MAX_JOYSTICK_ACCEL);
  frc::SmartDashboard::PutNumber("Drivebase/Config/MaxAngularAcceleration",
                                 MAX_ANGULAR_JOYSTICK_ACCEL);

  frc::SmartDashboard::PutData("Drivebase/Vision/Rotation Controller: ", &Rcontroller);

  frc::SmartDashboard::PutNumber("Drivebase/Config/MaxAngularAcceleration",
                                 MAX_ANGULAR_JOYSTICK_ACCEL);
  _gyro.Calibrate();
  Rcontroller.EnableContinuousInput(0_deg, 360_deg);
  frc::SmartDashboard::PutData("field", &_fieldDisplay);

  using namespace pathplanner;
  AutoBuilder::configureHolonomic(
      [this]() { return GetPose(); },  // Robot pose supplier
      [this](frc::Pose2d pose) {
        auto alliance = frc::DriverStation::GetAlliance();
        if (alliance) {
          if (alliance.value() == frc::DriverStation::Alliance::kBlue) {
            ResetGyroHeading(pose.Rotation().RotateBy(180_deg).Degrees());
          } else {
            ResetGyroHeading(pose.Rotation().Degrees());
          }
        }

        SetPose(pose);
      },  // Method to reset odometry (will be called if your auto has a starting pose)
      [this]() {
        return GetRobotRelativeSpeeds();
      },  // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      [this](frc::ChassisSpeeds speeds) {
        _sidewaysSpeedRequest = speeds.vy;  // TEST!
        _forwardSpeedRequest = speeds.vx;
        _rotationSpeedRequest = speeds.omega;
        _fieldOrientedRequest = false;
        // Drive(speeds.vx, speeds.vy, -speeds.omega, false);
      },  // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      HolonomicPathFollowerConfig(
          PIDConstants(2, 0.0, 0.0),    // Translation PID constants
          PIDConstants(0.5, 0.0, 0.0),  // Rotation PID constants
          MAX_VELOCITY,                 // Max module speed, in m/s
          432_mm,  // Drive base radius in meters. Distance from robot center to furthest module.
                   // NEEDS TO BE CHECKED AND MADE ACCURATE!!
          ReplanningConfig(
              false, false, 1_m,
              0.25_m)  // Default path replanning config. See the API for the options here
          ),

      []() {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
        auto alliance = frc::DriverStation::GetAlliance();
        if (alliance) {
          return alliance.value() == frc::DriverStation::Alliance::kRed;
        }
        std::cout << "Failed to detect alliance\n";
        return false;
      },
      this  // Reference to this subsystem to set requirements
  );
}

// This method will be called once per scheduler run
void SubDrivebase::Periodic() {
  auto loopStart = frc::GetTime();
  // Dashboard Displays:
  frc::SmartDashboard::PutNumber("Drivebase/heading", GetHeading().Degrees().value());
  frc::SmartDashboard::PutNumber("Drivebase/velocity", GetVelocity().value());

  frc::SmartDashboard::PutNumberArray("Drivebase/true swerve states",
                                      std::array{
                                          _frontLeft.GetAngle().Degrees().value(),
                                          _frontLeft.GetSpeed().value(),
                                          _frontRight.GetAngle().Degrees().value(),
                                          _frontRight.GetSpeed().value(),
                                          _backLeft.GetAngle().Degrees().value(),
                                          _backLeft.GetSpeed().value(),
                                          _backRight.GetAngle().Degrees().value(),
                                          _backRight.GetSpeed().value(),
                                      });

  frc::SmartDashboard::PutNumberArray("Drivebase/can coders swerve states",
                                      std::array{
                                          _frontLeft.GetCanCoderAngle().Degrees().value(),
                                          _frontLeft.GetSpeed().value(),
                                          _frontRight.GetCanCoderAngle().Degrees().value(),
                                          _frontRight.GetSpeed().value(),
                                          _backLeft.GetCanCoderAngle().Degrees().value(),
                                          _backLeft.GetSpeed().value(),
                                          _backRight.GetCanCoderAngle().Degrees().value(),
                                          _backRight.GetSpeed().value(),
                                      });

  _frontLeft.SendSensorsToDash();
  _frontRight.SendSensorsToDash();
  _backLeft.SendSensorsToDash();
  _backRight.SendSensorsToDash();

  Drive(_forwardSpeedRequest, _sidewaysSpeedRequest, _rotationSpeedRequest, _fieldOrientedRequest);
  frc::SmartDashboard::PutNumber("Drivebase/ x-axis translation speed request ",
                                 _forwardSpeedRequest.value());
  frc::SmartDashboard::PutNumber("Drivebase/ y-axis translation speed request ",
                                 _sidewaysSpeedRequest.value());
  frc::SmartDashboard::PutNumber("Drivebase/ rotation speed request ",
                                 _rotationSpeedRequest.value());
  frc::SmartDashboard::PutBoolean("Drivebase/ field oriented request", _fieldOrientedRequest);

  UpdateOdometry();
  frc::SmartDashboard::PutNumber("Drivebase/loop time (sec)", (frc::GetTime() - loopStart).value());
}

void SubDrivebase::SimulationPeriodic() {
  _frontLeft.UpdateSim(20_ms);
  _frontRight.UpdateSim(20_ms);
  _backLeft.UpdateSim(20_ms);
  _backRight.UpdateSim(20_ms);
}

frc2::CommandPtr SubDrivebase::JoystickDrive(frc2::CommandXboxController &controller,
                                             bool ignoreJoystickRotation) {
  return Run([this, &controller, ignoreJoystickRotation]
             {
    double deadband = 0.08;
    auto velocity =
        frc::SmartDashboard::GetNumber("Drivebase/Config/MaxVelocity", MAX_VELOCITY.value()) *
        1_mps;
    auto angularVelocity = frc::SmartDashboard::GetNumber("Drivebase/Config/MaxAngularVelocity",
                                                          MAX_ANGULAR_VELOCITY.value()) *
                           1_deg_per_s;
    static frc::SlewRateLimiter<units::scalar> _xspeedLimiter{MAX_JOYSTICK_ACCEL / 1_s};
    static frc::SlewRateLimiter<units::scalar> _yspeedLimiter{MAX_JOYSTICK_ACCEL / 1_s};
    static frc::SlewRateLimiter<units::scalar> _rotLimiter{MAX_ANGULAR_JOYSTICK_ACCEL / 1_s};
    auto forwardSpeed =
        _yspeedLimiter.Calculate(frc::ApplyDeadband(controller.GetLeftY(), deadband)) * velocity;
    auto rotationSpeed =
        _rotLimiter.Calculate(frc::ApplyDeadband(controller.GetRightX(), deadband)) *
        angularVelocity;
    auto sidewaysSpeed =
        _xspeedLimiter.Calculate(frc::ApplyDeadband(controller.GetLeftX(), deadband)) * velocity;

    // when optionalRotationContro is false,
    _sidewaysSpeedRequest = -sidewaysSpeed;
    _forwardSpeedRequest = -forwardSpeed;
    _fieldOrientedRequest = true;
    if (!ignoreJoystickRotation) {
      _rotationSpeedRequest = -rotationSpeed;
    } });
}

void SubDrivebase::Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
                         units::degrees_per_second_t rot, bool fieldRelative) {
  
  // Optionally convert speeds to field relative
  auto speeds = fieldRelative
                    ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetHeading())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot};

  // Discretize to get rid of translational drift while rotating
  constexpr bool inSim = frc::RobotBase::IsSimulation();
  speeds = frc::ChassisSpeeds::Discretize(speeds, inSim ? 200_ms : -200_ms);

  // Get states of all swerve modules
  auto states = _kinematics.ToSwerveModuleStates(speeds);

  // Set speed limit and apply speed limit to all modules
  _kinematics.DesaturateWheelSpeeds(
      &states,
      frc::SmartDashboard::GetNumber("Drivebase/Config/MaxVelocity", MAX_VELOCITY.value()) * 1_mps);

  // Setting modules from aquired states
  auto [fl, fr, bl, br] = states;

  frc::SmartDashboard::PutNumberArray("Drivebase/desired swerve states",
                                      std::array{
                                          fl.angle.Degrees().value(),
                                          fl.speed.value(),
                                          fr.angle.Degrees().value(),
                                          fr.speed.value(),
                                          bl.angle.Degrees().value(),
                                          bl.speed.value(),
                                          br.angle.Degrees().value(),
                                          br.speed.value(),
                                      });

  _frontLeft.SetDesiredState(fl);
  _frontRight.SetDesiredState(fr);
  _backLeft.SetDesiredState(bl);
  _backRight.SetDesiredState(br);

  // Check if robot is in simulation.
  // Manualy adjusting gyro by calculating rotation in simulator as gyro is not enabled in
  // simulation
  if (frc::RobotBase::IsSimulation()) {
    units::radian_t radPer20ms = rot * 20_ms;
    units::degree_t newHeading = GetHeading().RotateBy(radPer20ms).Degrees();
    _gyro.SetAngleAdjustment(-newHeading.value());  // negative to switch to CW from CCW
  }
}

void SubDrivebase::StopDriving() {
  _rotationSpeedRequest = 0_deg_per_s;
  _sidewaysSpeedRequest = 0_mps;
  _forwardSpeedRequest = 0_mps;
}

frc::ChassisSpeeds SubDrivebase::GetRobotRelativeSpeeds() {
  auto fl = _frontLeft.GetState();
  auto fr = _frontRight.GetState();
  auto bl = _backLeft.GetState();
  auto br = _backRight.GetState();
  return _kinematics.ToChassisSpeeds(fl, fr, bl, br);
}

// Syncs encoder values when the robot is turned on
void SubDrivebase::SyncSensors() {
  _frontLeft.SyncSensors();
  _frontRight.SyncSensors();
  _backLeft.SyncSensors();
  _backRight.SyncSensors();
  _gyro.Calibrate();

//Set config turn motors so it can run in auto init also. Had issues with parameters not being set on startup
  _frontLeft.ConfigTurnMotor();
  _frontRight.ConfigTurnMotor();
  _backLeft.ConfigTurnMotor();
  _backRight.ConfigTurnMotor();
}

frc2::CommandPtr SubDrivebase::SyncSensorBut() {
  return RunOnce([this] { SyncSensors(); });
}

frc::Rotation2d SubDrivebase::GetHeading() {
  return _gyro.GetRotation2d();
}

// Calculate robot's velocity over past time step (20 ms)
units::meters_per_second_t SubDrivebase::GetVelocity() {
  auto robotDisplacement =
      _prevPose.Translation().Distance(_poseEstimator.GetEstimatedPosition().Translation());
  return units::meters_per_second_t{robotDisplacement / 20_ms};
}

frc::SwerveDriveKinematics<4> SubDrivebase::GetKinematics() {
  return _kinematics;
}

// calculates the relative field location
void SubDrivebase::UpdateOdometry() {
  auto fl = _frontLeft.GetPosition();
  auto fr = _frontRight.GetPosition();
  auto bl = _backLeft.GetPosition();
  auto br = _backRight.GetPosition();

  _prevPose = _poseEstimator.GetEstimatedPosition();
  _poseEstimator.Update(GetHeading(), {fl, fr, bl, br});
  _fieldDisplay.SetRobotPose(_poseEstimator.GetEstimatedPosition());
}

void SubDrivebase::DriveToPose(frc::Pose2d targetPose) {
  DisplayPose("targetPose", targetPose);

  frc::Pose2d currentPosition = _poseEstimator.GetEstimatedPosition();
  double speedX = Xcontroller.Calculate(currentPosition.X().value(), targetPose.X().value());
  double speedY = Ycontroller.Calculate(currentPosition.Y().value(), targetPose.Y().value());
  double speedRot = Rcontroller.Calculate(currentPosition.Rotation().Radians(), targetPose.Rotation().Radians());

  speedX = std::clamp(speedX, -0.5, 0.5);
  speedY = std::clamp(speedY, -0.5, 0.5);
  speedRot = std::clamp(speedRot, -2.0, 2.0);

  // Drive speeds are relative to your alliance wall. Flip if we are on red,
  // since we are using global coordinates (blue alliance at 0,0)
  if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed && frc::RobotBase::IsReal()) {
    Drive(-speedX * 1_mps, -speedY * 1_mps, speedRot * 1_rad_per_s, true);
  } else {
    Drive(speedX * 1_mps, speedY * 1_mps, speedRot * 1_rad_per_s, true);
  }
}

void SubDrivebase::RotateToZero(units::degree_t rotationError) {
  double speedRot = Rcontroller.Calculate(rotationError, 0_deg);
  speedRot = std::clamp(speedRot, -2.0, 2.0);

  _rotationSpeedRequest = speedRot * 1_rad_per_s;
}

void SubDrivebase::TranslateToZero(units::degree_t translationError) {
  double speedX = Xcontroller.Calculate(translationError.value(), 0);

  speedX = std::clamp(speedX, -0.5, 0.5);
  _sidewaysSpeedRequest = speedX * 1_mps;
  _fieldOrientedRequest = false;
}

bool SubDrivebase::IsAtPose(frc::Pose2d pose) {
  auto currentPose = _poseEstimator.GetEstimatedPosition();
  auto rotError = currentPose.Rotation() - pose.Rotation();
  auto posError = currentPose.Translation().Distance(pose.Translation());

  if (units::math::abs(rotError.Degrees()) < 1_deg && posError < 1_cm) {
    return true;
  } else {
    return false;
  }
}

void SubDrivebase::ResetGyroHeading(units::degree_t startingAngle) {
  _gyro.Reset();
  _gyro.SetAngleAdjustment(startingAngle.value());
}

frc2::CommandPtr SubDrivebase::ResetGyroCmd() {
  return RunOnce([this] { ResetGyroHeading(); });
}

frc::Pose2d SubDrivebase::GetPose() {
  return _poseEstimator.GetEstimatedPosition();
}

void SubDrivebase::SetPose(frc::Pose2d pose) {
  auto fl = _frontLeft.GetPosition();
  auto fr = _frontRight.GetPosition();
  auto bl = _backLeft.GetPosition();
  auto br = _backRight.GetPosition();
  _poseEstimator.ResetPosition(GetHeading(), {fl, fr, bl, br}, pose);
}

void SubDrivebase::DisplayPose(std::string label, frc::Pose2d pose) {
  _fieldDisplay.GetObject(label)->SetPose(pose);
}

void SubDrivebase::UpdatePosition(frc::Pose2d robotPosition) {
  _poseEstimator.AddVisionMeasurement(robotPosition, 2_ms);
}

void SubDrivebase::DisplayTrajectory(std::string name, frc::Trajectory trajectory) {
  _fieldDisplay.GetObject(name)->SetTrajectory(trajectory);
}

void SubDrivebase::AddVisionMeasurement(frc::Pose2d pose, double ambiguity,
                                        units::second_t timeStamp) {
  frc::SmartDashboard::PutNumber("Timestamp", timeStamp.value());
  _poseEstimator.AddVisionMeasurement(pose, timeStamp);
}

void SubDrivebase::SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue mode) {
  _frontLeft.SetNeutralMode(mode);
  _frontRight.SetNeutralMode(mode);
  _backLeft.SetNeutralMode(mode);
  _backRight.SetNeutralMode(mode);
}

units::degree_t SubDrivebase::GetPitch() {
  return _gyro.GetPitch() * 1_deg;
}

frc2::CommandPtr SubDrivebase::WheelCharecterisationCmd(){
  static units::radian_t initialGyroHeading = 0_rad;
  static units::radian_t initialWheelDistance = 0_rad;

  return StartEnd(
      [this]{
        initialGyroHeading = GetHeading().Radians();
        // initialWheelDistance =
        //     (_frontRight.GetPosition().distance + _frontLeft.GetPosition().distance +
        //      _backRight.GetPosition().distance + _backLeft.GetPosition().distance) /
        //     4;
        initialWheelDistance = _frontRight.GetDrivenRotations();
        _forwardSpeedRequest = 0_mps;
        _sidewaysSpeedRequest = 0_mps;
        _rotationSpeedRequest = 15_deg_per_s;
      },
      [this] {
        units::meter_t drivebaseRadius = _frontLeftLocation.Norm();
        units::radian_t finalGyroHeading = GetHeading().Radians();
        // auto finalWheelDistance =
        //     (_frontRight.GetPosition().distance + _frontLeft.GetPosition().distance +
        //      _backRight.GetPosition().distance + _backLeft.GetPosition().distance) /
        //     4;
        units::radian_t finalWheelDistance = _frontRight.GetDrivenRotations();
        _rotationSpeedRequest = 0_deg_per_s;

        units::radian_t gyroDelta = finalGyroHeading - initialGyroHeading;
        units::radian_t wheelDistanceDelta = finalWheelDistance - initialWheelDistance;

        frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/CalcedWheelRadius",
                                       ((gyroDelta * drivebaseRadius) / wheelDistanceDelta).value());
        frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/Gyro", gyroDelta.value());
        frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/DrivebaseRadius", drivebaseRadius.value());
        frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/WheelDistance", wheelDistanceDelta.value());
      });
}