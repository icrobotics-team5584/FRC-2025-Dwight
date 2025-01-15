#include "subsystems/SubDrivebase.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <frc/RobotBase.h>
#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/DriverStation.h>
#include <units/time.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/config/RobotConfig.h>
#include "utilities/RobotLogs.h"
#include <frc/MathUtil.h>

SubDrivebase::SubDrivebase() {
  frc::SmartDashboard::PutData("Drivebase/Teleop PID/Rotation Controller", &_teleopRotationController);
  frc::SmartDashboard::PutData("Drivebase/Teleop PID/Translation Controller", &_teleopTranslationController);

  _teleopRotationController.EnableContinuousInput(0_deg, 360_deg);
  frc::SmartDashboard::PutData("field", &_fieldDisplay);

  using namespace pathplanner;
  AutoBuilder::configure(
      // Robot pose supplier
      [this]() { return GetPose(); },

      // Method to reset odometry (will be called if your auto has a starting pose)
      [this](frc::Pose2d pose) { SetPose(pose); },

      // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      [this]() { return GetRobotRelativeSpeeds(); },

      // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally
      // outputs individual module feedforwards
      [this](auto speeds, auto feedforwards) {
        if (feedforwards.robotRelativeForcesX.size() == 4 &&
            feedforwards.robotRelativeForcesY.size() == 4) {
          std::array<units::newton_t, 4> xForces = {
              feedforwards.robotRelativeForcesX[0], feedforwards.robotRelativeForcesX[1],
              feedforwards.robotRelativeForcesX[2], feedforwards.robotRelativeForcesX[3]};
          std::array<units::newton_t, 4> yForces = {
              feedforwards.robotRelativeForcesY[0], feedforwards.robotRelativeForcesY[1],
              feedforwards.robotRelativeForcesY[2], feedforwards.robotRelativeForcesY[3]};
          Drive(speeds.vx, speeds.vy, speeds.omega, false, xForces, yForces);
        } else {
          Drive(speeds.vx, speeds.vy, speeds.omega, false);
        }
      },
      // PID Feedback controller for translation and rotation
      _pathplannerController,

      // robot mass, MOT, wheel locations, etc
      RobotConfig::fromGUISettings(),

      // Boolean supplier that controls when the path will be mirrored for the red alliance
      // This will flip the path being followed to the red side of the field.
      // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
      []() {
        auto alliance = frc::DriverStation::GetAlliance();
        if (alliance) {
          Logger::Log("Drivebase/Pathplanner flipped to alliance", alliance.value());
          return alliance.value() == frc::DriverStation::Alliance::kRed;
        }
        Logger::Log("Drivebase/Pathplanner flipped to alliance",
                    "Failed to detect alliance, assuming blue");
        return false;
      },

      // Reference to this subsystem to set requirements
      this);
}

void SubDrivebase::Periodic() {
  auto loopStart = frc::GetTime();
  Logger::Log("Drivebase/heading", GetHeading());
  Logger::Log("Drivebase/velocity", GetVelocity());
  Logger::Log("Drivebase/Internal Encoder Swerve States",
              wpi::array{_frontLeft.GetState(), _frontRight.GetState(), _backLeft.GetState(),
                         _backRight.GetState()});
  Logger::Log("Drivebase/CANCoder Swerve States",
              wpi::array{_frontLeft.GetCANCoderState(), _frontRight.GetCANCoderState(),
                         _backLeft.GetCANCoderState(), _backRight.GetCANCoderState()});
  units::turn_t flRotations = _frontLeft.GetDrivenRotations();
  units::turn_t frRotations = _frontRight.GetDrivenRotations();
  units::turn_t blRotations = _backLeft.GetDrivenRotations();
  units::turn_t brRotations = _backRight.GetDrivenRotations();

  Logger::Log("Drivebase/DistanceDrivenRotations/fl", flRotations);
  Logger::Log("Drivebase/DistanceDrivenRotations/fr", frRotations);
  Logger::Log("Drivebase/DistanceDrivenRotations/bl", blRotations);
  Logger::Log("Drivebase/DistanceDrivenRotations/br", brRotations);

  Logger::Log("Drivebase/DistanceDriven/fl", (flRotations)*(0.04121451348939883*2*std::numbers::pi));
  Logger::Log("Drivebase/DistanceDriven/fr", (frRotations)*(0.04121451348939883*2*std::numbers::pi));
  Logger::Log("Drivebase/DistanceDriven/bl", (blRotations)*(0.04121451348939883*2*std::numbers::pi));
  Logger::Log("Drivebase/DistanceDriven/br", (brRotations)*(0.04121451348939883*2*std::numbers::pi));
  // 0.04121451348939883
  // 4.465m

  _frontLeft.SendSensorsToDash();
  _frontRight.SendSensorsToDash();
  _backLeft.SendSensorsToDash();
  _backRight.SendSensorsToDash();

  UpdateOdometry();
  frc::SmartDashboard::PutNumber("Drivebase/loop time (sec)", (frc::GetTime() - loopStart).value());
}

void SubDrivebase::SimulationPeriodic() {
  _frontLeft.UpdateSim(20_ms);
  _frontRight.UpdateSim(20_ms);
  _backLeft.UpdateSim(20_ms);
  _backRight.UpdateSim(20_ms);

  // Adjust gyro angle
  auto rotSpeed = _kinematics
                      .ToChassisSpeeds(_frontLeft.GetState(), _frontRight.GetState(),
                                       _backLeft.GetState(), _backRight.GetState())
                      .omega;
  units::radian_t changeInRot = rotSpeed * 20_ms;
  units::degree_t newHeading = GetGyroAngle().RotateBy(changeInRot).Degrees();
  _gyro.SetAngleAdjustment(-newHeading.value());  // negative to switch to CW from CCW

  auto fl = _frontLeft.GetPosition();
  auto fr = _frontRight.GetPosition();
  auto bl = _backLeft.GetPosition();
  auto br = _backRight.GetPosition();

  _simPoseEstimator.Update(GetGyroAngle(), {fl, fr, bl, br});
  DisplayPose("Sim final pose v3 for real", _simPoseEstimator.GetEstimatedPosition());


}

void SubDrivebase::SetPathplannerRotationFeedbackSource(
    std::function<units::turns_per_second_t()> rotationFeedbackSource) {
  pathplanner::PPHolonomicDriveController::overrideRotationFeedback(rotationFeedbackSource);
}

void SubDrivebase::ResetPathplannerRotationFeedbackSource() {
  pathplanner::PPHolonomicDriveController::clearRotationFeedbackOverride();
}

frc::ChassisSpeeds SubDrivebase::CalcJoystickSpeeds(frc2::CommandXboxController& controller) {
  std::string path = "Drivebase/Config/";
  auto deadband = Logger::Tune(path + "Joystick Deadband", JOYSTICK_DEADBAND);
  auto maxVelocity = Logger::Tune(path + "Max Velocity", MAX_VELOCITY);
  auto maxAngularVelocity = Logger::Tune(path + "Max Angular Velocity", MAX_ANGULAR_VELOCITY);
  auto maxJoystickAccel = Logger::Tune(path + "Max Joystick Accel", MAX_JOYSTICK_ACCEL);
  auto maxAngularJoystickAccel =
      Logger::Tune(path + "Max Joystick Angular Accel", MAX_ANGULAR_JOYSTICK_ACCEL);

  // Recreate slew rate limiters if limits have changed
  if (maxJoystickAccel != _tunedMaxJoystickAccel) {
    _xStickLimiter = frc::SlewRateLimiter<units::scalar>{maxJoystickAccel / 1_s};
    _yStickLimiter = frc::SlewRateLimiter<units::scalar>{maxJoystickAccel / 1_s};
    _tunedMaxJoystickAccel = maxJoystickAccel;
  }
  if (maxAngularJoystickAccel != _tunedMaxAngularJoystickAccel) {
    _rotStickLimiter = frc::SlewRateLimiter<units::scalar>{maxAngularJoystickAccel / 1_s};
    _tunedMaxAngularJoystickAccel = maxAngularJoystickAccel;
  }

  // Apply deadbands
  double forwardStick = frc::ApplyDeadband(-controller.GetLeftY(), deadband);
  double sidewaysStick = frc::ApplyDeadband(-controller.GetLeftX(), deadband);
  double rotationStick = frc::ApplyDeadband(-controller.GetRightX(), deadband);

  // Apply joystick rate limits
  auto forwardSpeed = _yStickLimiter.Calculate(forwardStick) * maxVelocity;
  auto sidewaysSpeed = _xStickLimiter.Calculate(sidewaysStick) * maxVelocity;
  auto rotationSpeed = _rotStickLimiter.Calculate(rotationStick) * maxAngularVelocity;

  return frc::ChassisSpeeds{forwardSpeed, sidewaysSpeed, rotationSpeed};
}

frc2::CommandPtr SubDrivebase::JoystickDrive(frc2::CommandXboxController& controller) {
  return Drive([this, &controller] { return CalcJoystickSpeeds(controller); }, true);
}

frc2::CommandPtr SubDrivebase::Drive(std::function<frc::ChassisSpeeds()> speeds,
                                     bool fieldOriented) {
  return Run([this, speeds, fieldOriented] {
           auto speedVals = speeds();
           Drive(speedVals.vx, speedVals.vy, speedVals.omega, fieldOriented);
         })
      .FinallyDo([this] { Drive(0_mps, 0_mps, 0_deg_per_s, false); });
}


void SubDrivebase::DriveToPose(frc::Pose2d targetPose) {
 // DisplayPose("targetPose", targetPose);

  frc::Pose2d currentPosition = _poseEstimator.GetEstimatedPosition();
  double speedX = _teleopTranslationController.Calculate(currentPosition.X().value(), targetPose.X().value());
  double speedY = _teleopTranslationController.Calculate(currentPosition.Y().value(), targetPose.Y().value());
  double speedRot = _teleopRotationController.Calculate(currentPosition.Rotation().Radians(), targetPose.Rotation().Radians());

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

void SubDrivebase::Drive(
  units::meters_per_second_t xSpeed, 
  units::meters_per_second_t ySpeed,
  units::turns_per_second_t rot, 
  bool fieldRelative,
  std::optional<std::array<units::newton_t, 4>> xForceFeedforwards,
  std::optional<std::array<units::newton_t, 4>> yForceFeedforwards
) {
  // Optionally convert speeds to field relative
  auto speeds = fieldRelative
                    ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeed, ySpeed, rot, GetHeading())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot};

  // Discretize to get rid of translational drift while rotating
  constexpr bool inSim = frc::RobotBase::IsSimulation();
  speeds = frc::ChassisSpeeds::Discretize(speeds, inSim ? 20_ms : 60_ms);

  // Get states of all swerve modules
  auto states = _kinematics.ToSwerveModuleStates(speeds);

  // Set speed limit and apply speed limit to all modules
  _kinematics.DesaturateWheelSpeeds(
      &states,
      frc::SmartDashboard::GetNumber("Drivebase/Config/MaxVelocity", MAX_VELOCITY.value()) * 1_mps);
  
  // Extract force feedforwards
  std::array<units::newton_t, 4> defaults{0_N, 0_N, 0_N, 0_N};
  auto [flXForce, frXForce, blXForce, brXForce] = xForceFeedforwards.value_or(defaults);
  auto [flYForce, frYForce, blYForce, brYForce] = yForceFeedforwards.value_or(defaults);

  // Setting modules from aquired states
  Logger::Log("Drivebase/Desired Swerve States", states);
  auto [fl, fr, bl, br] = states;
  _frontLeft.SetDesiredState(fl, flXForce, flYForce);
  _frontRight.SetDesiredState(fr, frXForce, frYForce);
  _backLeft.SetDesiredState(bl, blXForce, blYForce);
  _backRight.SetDesiredState(br, brXForce, brYForce);
}

frc::ChassisSpeeds SubDrivebase::GetRobotRelativeSpeeds() {
  auto fl = _frontLeft.GetState();
  auto fr = _frontRight.GetState();
  auto bl = _backLeft.GetState();
  auto br = _backRight.GetState();
  return _kinematics.ToChassisSpeeds(fl, fr, bl, br);
}

void SubDrivebase::SyncSensors() {
  _frontLeft.SyncSensors();
  _frontRight.SyncSensors();
  _backLeft.SyncSensors();
  _backRight.SyncSensors();

  // config turn motors so it can run in auto init also. Had issues with parameters not being set on startup
  _frontLeft.ConfigTurnMotor();
  _frontRight.ConfigTurnMotor();
  _backLeft.ConfigTurnMotor();
  _backRight.ConfigTurnMotor();
}

frc2::CommandPtr SubDrivebase::SyncSensorBut() {
  return RunOnce([this] { SyncSensors(); });
}

frc::Rotation2d SubDrivebase::GetHeading() {
  return _poseEstimator.GetEstimatedPosition().Rotation();
}

frc::Rotation2d SubDrivebase::GetGyroAngle() {
  return _gyro.GetRotation2d();
}

units::meters_per_second_t SubDrivebase::GetVelocity() {
  // Use pythag to find velocity from x and y components
  auto speeds = _kinematics.ToChassisSpeeds(_frontLeft.GetState(), _frontRight.GetState(),
                                            _backLeft.GetState(), _backRight.GetState());
  namespace m = units::math;
  return m::sqrt(m::pow<2>(speeds.vx) + m::pow<2>(speeds.vy));
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

  _poseEstimator.Update(GetGyroAngle(), {fl, fr, bl, br});
  _fieldDisplay.SetRobotPose(_poseEstimator.GetEstimatedPosition());
}

frc::ChassisSpeeds SubDrivebase::CalcDriveToPoseSpeeds(frc::Pose2d targetPose) {
  // Find current and target values
  DisplayPose("WERTY/targetPose", targetPose);
  double targetXMeters = targetPose.X().value();
  double targetYMeters = targetPose.Y().value();
  units::turn_t targetRotation = targetPose.Rotation().Radians();
  frc::Pose2d currentPosition = GetPose();
  double currentXMeters = currentPosition.X().value();
  double currentYMeters = currentPosition.Y().value();
  units::turn_t currentRotation = currentPosition.Rotation().Radians();

  // Use PID controllers to calculate speeds
  auto xSpeed = _teleopTranslationController.Calculate(currentXMeters, targetXMeters) * 1_mps;
  auto ySpeed = _teleopTranslationController.Calculate(currentYMeters, targetYMeters) * 1_mps;
  auto rSpeed = CalcRotateSpeed(currentRotation - targetRotation);

  // Clamp to max velocity
  xSpeed = units::math::min(xSpeed, MAX_VELOCITY);
  xSpeed = units::math::max(xSpeed, -MAX_VELOCITY);
  ySpeed = units::math::min(ySpeed, MAX_VELOCITY);
  ySpeed = units::math::max(ySpeed, -MAX_VELOCITY);

  frc::SmartDashboard::PutNumber("CalcDriveLogs/xSpeed", -xSpeed.value());
  frc::SmartDashboard::PutNumber("CalcDriveLogs/ySpeed", ySpeed.value());
  frc::SmartDashboard::PutNumber("CalcDriveLogs/rSpeed", rSpeed.value());
  frc::SmartDashboard::PutNumber("CalcDriveLogs/targetXMeters", targetXMeters);
  frc::SmartDashboard::PutNumber("CalcDriveLogs/targetYMeters", targetYMeters);
  frc::SmartDashboard::PutNumber("CalcDriveLogs/currentXMeters", currentXMeters);
  frc::SmartDashboard::PutNumber("CalcDriveLogs/currentYMeters", currentYMeters);
  frc::SmartDashboard::PutNumber("CalcDriveLogs/currentRotation", currentRotation.value());
  return frc::ChassisSpeeds{ xSpeed, ySpeed, rSpeed};
}

units::turns_per_second_t SubDrivebase::CalcRotateSpeed(units::turn_t rotationError) { 
  auto omega = _teleopRotationController.Calculate(rotationError, 0_deg) * 1_rad_per_s;
  omega = units::math::min(omega, MAX_ANGULAR_VELOCITY);
  omega = units::math::max(omega, -MAX_ANGULAR_VELOCITY);
  return omega;
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
  return RunOnce([this] { _poseEstimator.ResetRotation(0_deg); });
}

frc::Pose2d SubDrivebase::GetPose() {
  return _poseEstimator.GetEstimatedPosition();
}

frc::Pose2d SubDrivebase::GetSimPose() {
  return _simPoseEstimator.GetEstimatedPosition();
}
void SubDrivebase::SetPose(frc::Pose2d pose) {
  auto fl = _frontLeft.GetPosition();
  auto fr = _frontRight.GetPosition();
  auto bl = _backLeft.GetPosition();
  auto br = _backRight.GetPosition();
  _poseEstimator.ResetPosition(GetGyroAngle(), {fl, fr, bl, br}, pose);
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
  _poseEstimator.AddVisionMeasurement(frc::Pose2d{pose.X(), pose.Y(), GetPose().Rotation()}, timeStamp);
}

void SubDrivebase::SetNeutralMode(bool mode) {
  _frontLeft.SetBreakMode(mode);
  _frontRight.SetBreakMode(mode);
  _backLeft.SetBreakMode(mode);
  _backRight.SetBreakMode(mode);
}

units::degree_t SubDrivebase::GetPitch() {
  return _gyro.GetPitch() * 1_deg;
}

frc2::CommandPtr SubDrivebase::WheelCharecterisationCmd() {
  static units::radian_t prevGyroAngle = 0_rad;
  static units::radian_t gyroAccumulator = 0_rad;
  static units::radian_t FRinitialWheelDistance = 0_rad;
  static units::radian_t FLinitialWheelDistance = 0_rad;
  static units::radian_t BRinitialWheelDistance = 0_rad;
  static units::radian_t BLinitialWheelDistance = 0_rad;

  return RunOnce([this] {
           prevGyroAngle = 0_rad;
           gyroAccumulator = 0_rad;
           FRinitialWheelDistance = _frontRight.GetDrivenRotations();
           FLinitialWheelDistance = _frontLeft.GetDrivenRotations();
           BRinitialWheelDistance = _backRight.GetDrivenRotations();
           BLinitialWheelDistance = _backLeft.GetDrivenRotations(); 

         })
      .AndThen(Drive([] { return frc::ChassisSpeeds{0_mps, 0_mps, -15_deg_per_s}; }, false))
      .AlongWith(frc2::cmd::Run([this] {
        //units::radian_t curGyroAngle = GetHeading().Radians(); using GetGyroAngle() instead
        units::radian_t curGyroAngle = GetGyroAngle().Radians();
        gyroAccumulator = gyroAccumulator + frc::AngleModulus((prevGyroAngle - curGyroAngle));
        prevGyroAngle = curGyroAngle;
        frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/GyroAccum", gyroAccumulator.value());
        frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/GyroCur", curGyroAngle.value());
        frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/GyroPrev", prevGyroAngle.value());
      }))
      .FinallyDo([this] {
        units::meter_t drivebaseRadius = _frontLeftLocation.Norm();

        units::radian_t FRfinalWheelDistance = _frontRight.GetDrivenRotations();
        units::radian_t FLfinalWheelDistance = _frontLeft.GetDrivenRotations();
        units::radian_t BRfinalWheelDistance = _backRight.GetDrivenRotations();
        units::radian_t BLfinalWheelDistance = _backLeft.GetDrivenRotations();

        units::radian_t FRdelta = units::math::abs(FRfinalWheelDistance - FRinitialWheelDistance);
        units::radian_t FLdelta = units::math::abs(FLfinalWheelDistance - FLinitialWheelDistance);
        units::radian_t BRdelta = units::math::abs(BRfinalWheelDistance - BRinitialWheelDistance);
        units::radian_t BLdelta = units::math::abs(BLfinalWheelDistance - BLinitialWheelDistance);



        units::radian_t avgWheelDelta = (FRdelta + FLdelta + BRdelta + BLdelta) / 4.0;

        frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/CalcedWheelRadius",
                                       ((gyroAccumulator * drivebaseRadius) / avgWheelDelta).value());
        frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/DrivebaseRadius", drivebaseRadius.value());
        frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/WheelDistance", avgWheelDelta.value());

        // frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/FLinitialWheelDistance", FLinitialWheelDistance.value());
        // frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/FRinitialWheelDistance", FRinitialWheelDistance.value());
        // frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/BLinitialWheelDistance", BLinitialWheelDistance.value());
        // frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/BRinitialWheelDistance", BRinitialWheelDistance.value());

        // frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/FLfinalWheelDistance", FLfinalWheelDistance.value());
        // frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/FRfinalWheelDistance", FRfinalWheelDistance.value());
        // frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/BLfinalWheelDistance", BLfinalWheelDistance.value());
        // frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/BRfinalWheelDistance", BRfinalWheelDistance.value());

        frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/FLdelta", FLdelta.value());
        frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/FRdelta", FRdelta.value());
        frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/BLdelta", BLdelta.value());
        frc::SmartDashboard::PutNumber("Drivebase/WheelCharacterisation/BRdelta", BRdelta.value());
      });
}
