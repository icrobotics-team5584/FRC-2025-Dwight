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


SubDrivebase::SubDrivebase() {
  frc::SmartDashboard::PutData("Drivebase/Teleop PID/Rotation Controller", &_teleopRotationController);
  frc::SmartDashboard::PutData("Drivebase/Teleop PID/Translation Controller", &_teleopTranslationController);
  ConfigPigeon2();
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
        double _voltageFFscaler = 2.0; //Logger::Tune("drivebase/volatageFFscaler", 1.0); // this a scaler for the voltageFF
        if (feedforwards.robotRelativeForcesX.size() == 4 &&
            feedforwards.robotRelativeForcesY.size() == 4) {
          std::array<units::newton_t, 4> xForces = {
              (feedforwards.robotRelativeForcesX[0]/_voltageFFscaler), (feedforwards.robotRelativeForcesX[1]/_voltageFFscaler),
              (feedforwards.robotRelativeForcesX[2]/_voltageFFscaler), (feedforwards.robotRelativeForcesX[3]/_voltageFFscaler)};
          std::array<units::newton_t, 4> yForces = {
              (feedforwards.robotRelativeForcesY[0]/_voltageFFscaler), (feedforwards.robotRelativeForcesY[1]/_voltageFFscaler),
              (feedforwards.robotRelativeForcesY[2]/_voltageFFscaler), (feedforwards.robotRelativeForcesY[3]/_voltageFFscaler)};
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
  Logger::Log("Drivebase/Pigeon raw angle", _gyro.GetYaw().GetValue().value());
  Logger::Log("Drivebase/Pigeon raw Rotation2d", _gyro.GetRotation2d().Degrees());

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
  _gyro.SetYaw(newHeading);

  auto fl = _frontLeft.GetPosition();
  auto fr = _frontRight.GetPosition();
  auto bl = _backLeft.GetPosition();
  auto br = _backRight.GetPosition();

  _simPoseEstimator.Update(GetGyroAngle(), {fl, fr, bl, br});
  _simPoseEstimator.ResetRotation(GetPose().Rotation());
  DisplayPose("Sim final pose v3 for real", _simPoseEstimator.GetEstimatedPosition());
}

void SubDrivebase::SetPathplannerRotationFeedbackSource(
    std::function<units::turns_per_second_t()> rotationFeedbackSource) {
  pathplanner::PPHolonomicDriveController::overrideRotationFeedback(rotationFeedbackSource);
}

void SubDrivebase::ResetPathplannerRotationFeedbackSource() {
  pathplanner::PPHolonomicDriveController::clearRotationFeedbackOverride();
}

void SubDrivebase::ConfigPigeon2(){
  _gyroConfig.MountPose.MountPosePitch = 0_deg;  
  _gyroConfig.MountPose.MountPoseRoll = 0_deg;  
  _gyroConfig.MountPose.MountPoseYaw = 0_deg;
  _gyro.GetConfigurator().Apply(_gyroConfig);
}

frc::ChassisSpeeds SubDrivebase::CalcJoystickSpeeds(frc2::CommandXboxController& controller) {
  std::string configPath = "Drivebase/Config/";
  auto deadband = Logger::Tune(configPath + "Joystick Deadband", JOYSTICK_DEADBAND);
  auto maxVelocity = Logger::Tune(configPath + "Max Velocity", MAX_VELOCITY);
  auto maxAngularVelocity = Logger::Tune(configPath + "Max Angular Velocity", MAX_ANGULAR_VELOCITY);
  auto maxJoystickAccel = Logger::Tune(configPath + "Max Joystick Accel", MAX_JOYSTICK_ACCEL);
  auto maxAngularJoystickAccel = Logger::Tune(configPath + "Max Joystick Angular Accel", MAX_ANGULAR_JOYSTICK_ACCEL);
  auto translationRScaling = Logger::Tune(configPath + "Translation R-value Scaling", TRANSLATION_R_SCALING);
  auto rotationRScaling = Logger::Tune(configPath + "Rotation R-value Scaling", ROTATION_R_SCALING);

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
  double rawTranslationY = frc::ApplyDeadband(-controller.GetLeftY(), deadband);
  double rawTranslationX = frc::ApplyDeadband(-controller.GetLeftX(), deadband);
  double rawRotation = frc::ApplyDeadband(-controller.GetRightX(), deadband);

  // Convert cartesian (x, y) translation stick coordinates to polar (R, theta) and scale R-value
  double rawTranslationR = std::min(1.0, sqrt(pow(rawTranslationX, 2) + pow(rawTranslationY, 2)));
  double translationTheta = atan2(rawTranslationY, rawTranslationX);
  double scaledTranslationR = pow(rawTranslationR, translationRScaling);

  // Convert polar coordinates (with scaled R-value) back to cartesian; scale rotation as well
  double scaledTranslationY = scaledTranslationR*sin(translationTheta);
  double scaledTranslationX = scaledTranslationR*cos(translationTheta);

  double scaledRotation;
  if (rawRotation >= 0) { scaledRotation = pow(rawRotation, rotationRScaling); }
  else { scaledRotation = std::copysign(pow(abs(rawRotation), rotationRScaling), rawRotation); }

  // Apply joystick rate limits and calculate speed
  auto forwardSpeed = _yStickLimiter.Calculate(scaledTranslationY) * maxVelocity;
  auto sidewaysSpeed = _xStickLimiter.Calculate(scaledTranslationX) * maxVelocity;
  auto rotationSpeed = _rotStickLimiter.Calculate(scaledRotation) * maxAngularVelocity;

  // Dashboard things
  frc::SmartDashboard::PutNumber("Drivebase/Joystick Scaling/rawTranslationY", rawTranslationY);
  frc::SmartDashboard::PutNumber("Drivebase/Joystick Scaling/rawTranslationX", rawTranslationX);
  frc::SmartDashboard::PutNumber("Drivebase/Joystick Scaling/rawTranslationR", rawTranslationR);
  frc::SmartDashboard::PutNumber("Drivebase/Joystick Scaling/translationTheta (degrees)", translationTheta*(180/3.141592653589793238463)); //Multiply by 180/pi to convert radians to degrees
  frc::SmartDashboard::PutNumber("Drivebase/Joystick Scaling/scaledTranslationR", scaledTranslationR);
  frc::SmartDashboard::PutNumber("Drivebase/Joystick Scaling/scaledTranslationY", scaledTranslationY);
  frc::SmartDashboard::PutNumber("Drivebase/Joystick Scaling/scaledTranslationX", scaledTranslationX);
  frc::SmartDashboard::PutNumber("Drivebase/Joystick Scaling/rawRotation", rawRotation);
  frc::SmartDashboard::PutNumber("Drivebase/Joystick Scaling/scaledRotation", scaledRotation);

  return frc::ChassisSpeeds{forwardSpeed, sidewaysSpeed, rotationSpeed};
}

frc2::CommandPtr SubDrivebase::JoystickDrive(frc2::CommandXboxController& controller) {
  return Drive([this, &controller] { return CalcJoystickSpeeds(controller); }, true);
}

frc2::CommandPtr SubDrivebase::RobotCentricDrive(frc2::CommandXboxController& controller) {
  return {SubDrivebase::GetInstance().Drive([this, &controller] 
  {
    auto speeds = CalcJoystickSpeeds(controller);
    std::swap(speeds.vx, speeds.vy);

    return speeds;
  }, false)};
}

frc2::CommandPtr SubDrivebase::Drive(std::function<frc::ChassisSpeeds()> speeds,
                                     bool fieldOriented) {
  return Run([this, speeds, fieldOriented] {
           auto speedVals = speeds();
           Drive(speedVals.vx, speedVals.vy, speedVals.omega, fieldOriented);
         })
      .FinallyDo([this] { Drive(0_mps, 0_mps, 0_deg_per_s, false); });
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
      frc::SmartDashboard::GetNumber("Drivebase/Config/Max Velocity", MAX_VELOCITY.value()) * 1_mps);
  

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
  Logger::Log("Drivebase/speeds/vx", speeds.vx);
  Logger::Log("Drivebase/speeds/vy", speeds.vy);
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
  
  // frc::PIDController _driveToPoseTranslationController{1.7,0.5,0.0};
  // frc::ProfiledPIDController<units::radian> _driveToPoseRotationController{3,0.5,0.2, {MAX_ANGULAR_VELOCITY, MAX_ANG_ACCEL}};
  // auto xSpeed = _driveToPoseTranslationController.Calculate(currentXMeters, targetXMeters) * 1_mps;
  // auto ySpeed = _driveToPoseTranslationController.Calculate(currentYMeters, targetYMeters) * 1_mps;
  // auto rSpeed = _driveToPoseRotationController.Calculate(currentRotation - targetRotation, 0_deg) * 1_rad_per_s; 

  // Clamp to max velocity & angular velocity
  xSpeed = units::math::min(xSpeed, MAX_DRIVE_TO_POSE_VELOCITY); //Max_Velocity
  xSpeed = units::math::max(xSpeed, -MAX_DRIVE_TO_POSE_VELOCITY);
  ySpeed = units::math::min(ySpeed, MAX_DRIVE_TO_POSE_VELOCITY);
  ySpeed = units::math::max(ySpeed, -MAX_DRIVE_TO_POSE_VELOCITY);
  // rSpeed = units::math::min(rSpeed, MAX_ANGULAR_VELOCITY);
  // rSpeed = units::math::max(rSpeed, -MAX_ANGULAR_VELOCITY);

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
  auto velocity = GetVelocity();
  DisplayPose("current pose", currentPose);
  DisplayPose("target pose", pose);
  
  frc::SmartDashboard::PutNumber("Drivebase/rotError", units::math::abs(rotError.Degrees()).value());
  frc::SmartDashboard::PutNumber("Drivebase/posError", posError.value());

  frc::SmartDashboard::PutBoolean("Drivebase/IsAtPose", units::math::abs(rotError.Degrees()) < 1_deg && posError < 2_cm);

  if (units::math::abs(rotError.Degrees()) < 1_deg && posError < 2_cm && velocity < 0.0001_mps
  ) {
    return true;
  } else {
    return false;
  }
}

void SubDrivebase::ResetGyroHeading(units::degree_t startingAngle) {
  _gyro.Reset();
  _gyro.SetYaw(startingAngle);
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
  _simPoseEstimator.ResetPosition(GetGyroAngle(), {fl, fr, bl,  br}, pose);
}

void SubDrivebase::DisplayPose(std::string label, frc::Pose2d pose) {
  _fieldDisplay.GetObject(label)->SetPose(pose);
}

void SubDrivebase::DisplayTrajectory(std::string name, frc::Trajectory trajectory) {
  _fieldDisplay.GetObject(name)->SetTrajectory(trajectory);
}

void SubDrivebase::AddVisionMeasurement(frc::Pose2d pose, double ambiguity,
                                        units::second_t timeStamp) {
  _poseEstimator.AddVisionMeasurement(frc::Pose2d{pose.X(), pose.Y(), GetPose().Rotation()}, timeStamp);
}

void SubDrivebase::SetNeutralMode(bool mode) {
  _frontLeft.SetBreakMode(mode);
  _frontRight.SetBreakMode(mode);
  _backLeft.SetBreakMode(mode);
  _backRight.SetBreakMode(mode);
}

units::degree_t SubDrivebase::GetPitch() {
  return (_gyro.GetPitch().GetValue());
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