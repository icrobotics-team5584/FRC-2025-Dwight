#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/filter/SlewRateLimiter.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <numbers>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include "Constants.h"
#include "utilities/SwerveModule.h"
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <frc/DigitalInput.h>
#include "utilities/BotVars.h"
#include "utilities/RobotLogs.h"
#include <ctre/phoenix6/core/CorePigeon2.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>

class SubDrivebase : public frc2::SubsystemBase {
 public:
  SubDrivebase();
  static SubDrivebase& GetInstance() {
    static SubDrivebase inst;
    return inst;
  }
  void Periodic() override;
  void SimulationPeriodic() override;

  // Instantaneous functions
  void AddVisionMeasurement(frc::Pose2d pose, units::second_t timeStamp, wpi::array<double,3> dev);
  void ResetGyroHeading(units::degree_t startingAngle = 0_deg);
  void DisplayTrajectory(std::string name, frc::Trajectory trajectory);
  void SetBrakeMode(bool mode);
  void SetPose(frc::Pose2d pose);
  void DisplayPose(std::string label, frc::Pose2d pose);
  void UpdateOdometry();
  void SyncSensors();
  void SetPathplannerRotationFeedbackSource(
      std::function<units::turns_per_second_t()> rotationFeedbackSource);
  void ResetPathplannerRotationFeedbackSource();
  void ConfigPigeon2();

  // Getters
  bool IsAtPose(frc::Pose2d pose);
  frc2::Trigger CheckCoastButton();
  frc2::Trigger IsTipping();
  
  frc::ChassisSpeeds CalcDriveToPoseSpeeds(frc::Pose2d targetPose);
  frc::ChassisSpeeds CalcJoystickSpeeds(frc2::CommandXboxController& controller);
  units::turns_per_second_t CalcRotateSpeed(units::turn_t rotationError);
  units::degree_t GetPitch();
  units::degree_t GetRoll();
  frc::Pose2d GetPose();
  frc::Pose2d GetSimPose();
  frc::Rotation2d GetHeading();  // Heading as recorded by the pose estimator (matches field orientation)
  frc::Rotation2d GetGyroAngle();  // Heading as recorded by the gyro (zero is direction when switched on)
  frc::Rotation2d GetAllianceRelativeGyroAngle();
  units::meters_per_second_t GetVelocity();
  frc::SwerveDriveKinematics<4> GetKinematics();
  frc::ChassisSpeeds GetRobotRelativeSpeeds();

  // Commands
  frc2::CommandPtr JoystickDrive(frc2::CommandXboxController& controller);
  frc2::CommandPtr WheelCharecterisationCmd();
  frc2::CommandPtr Drive(std::function<frc::ChassisSpeeds()> speeds, bool fieldOriented);
  frc2::CommandPtr RobotCentricDrive(frc2::CommandXboxController& controller);
  void DriveToPose(frc::Pose2d targetPose);
  frc2::CommandPtr SyncSensorBut();
  frc2::CommandPtr ResetGyroCmd();
  frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction) {
    return _sysIdRoutine.Quasistatic(direction);
  }
  frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction) {
    return _sysIdRoutine.Dynamic(direction);
  }

  // Constants
  static constexpr units::meters_per_second_t MAX_VELOCITY = 5_mps;
  static constexpr units::meters_per_second_t MAX_DRIVE_TO_POSE_VELOCITY = 1_mps;
  static constexpr units::turns_per_second_t MAX_ANGULAR_VELOCITY =
      290_deg_per_s;  // CHANGE TO 720\[]

  static constexpr units::turns_per_second_squared_t MAX_ANG_ACCEL{std::numbers::pi};

  static constexpr double MAX_JOYSTICK_ACCEL = 5;
  static constexpr double MAX_ANGULAR_JOYSTICK_ACCEL = 3;
  static constexpr double JOYSTICK_DEADBAND = 0.08;
  static constexpr double TRANSLATION_R_SCALING = 2;  // Set to 1 for linear scaling
  static constexpr double ROTATION_R_SCALING = 1;     // Set to 1 for linear scaling

 private:
  void Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
             units::turns_per_second_t rot, bool fieldRelative,
             std::optional<std::array<units::newton_t, 4>> xForceFeedforwards = std::nullopt,
             std::optional<std::array<units::newton_t, 4>> yForceFeedforwards = std::nullopt);

  ctre::phoenix6::configs::Pigeon2Configuration _gyroConfig;
  ctre::phoenix6::hardware::Pigeon2 _gyro{canid::PIGEON_2};

  // Swerve modules
  frc::Translation2d _frontLeftLocation{+0.281_m, +0.281_m};
  frc::Translation2d _frontRightLocation{+0.281_m, -0.281_m};
  frc::Translation2d _backLeftLocation{-0.281_m, +0.281_m};
  frc::Translation2d _backRightLocation{-0.281_m, -0.281_m};

  const units::turn_t FRONT_RIGHT_MAG_OFFSET = //
      BotVars::Choose(-0.37451171875, -0.515380859375) * 1_tr;
  const units::turn_t FRONT_LEFT_MAG_OFFSET = //
      BotVars::Choose(-0.94091796875, -0.172607421875) * 1_tr;
  const units::turn_t BACK_RIGHT_MAG_OFFSET = //
      BotVars::Choose(-0.46435546875, -0.395263671875) * 1_tr;
  const units::turn_t BACK_LEFT_MAG_OFFSET =
      BotVars::Choose(-0.353515625, -0.94921875) * 1_tr;

  frc::DigitalInput _toggleBrakeCoast{dio::BRAKE_COAST_BUTTON};

  SwerveModule _frontLeft{canid::DRIVEBASE_FRONT_LEFT_DRIVE, canid::DRIVEBASE_FRONT_LEFT_TURN,
                          canid::DRIVEBASE_FRONT_LEFT_ENCODER, (FRONT_LEFT_MAG_OFFSET)};
  SwerveModule _frontRight{canid::DRIVEBASE_FRONT_RIGHT_DRIVE, canid::DRIVEBASE_FRONT_RIGHT_TURN,
                           canid::DRIVEBASE_FRONT_RIGHT_ENCODER, (FRONT_RIGHT_MAG_OFFSET)};
  SwerveModule _backLeft{canid::DRIVEBASE_BACK_LEFT_DRIVE, canid::DRIVEBASE_BACK_LEFT_TURN,
                         canid::DRIVEBASE_BACK_LEFT_ENCODER, (BACK_LEFT_MAG_OFFSET)};
  SwerveModule _backRight{canid::DRIVEBASE_BACK_RIGHT_DRIVE, canid::DRIVEBASE_BACK_RIGHT_TURN,
                          canid::DRIVEBASE_BACK_RIGHT_ENCODER, (BACK_RIGHT_MAG_OFFSET)};

  // Control objects
  frc::SwerveDriveKinematics<4> _kinematics{_frontLeftLocation, _frontRightLocation,
                                            _backLeftLocation, _backRightLocation};

  frc::PIDController _teleopTranslationController{1.7, 0.0, 0.0};
  frc::ProfiledPIDController<units::radian> _teleopRotationController{
      3, 0, 0.2, {MAX_ANGULAR_VELOCITY, MAX_ANG_ACCEL}};
  std::shared_ptr<pathplanner::PPHolonomicDriveController> _pathplannerController =
      std::make_shared<pathplanner::PPHolonomicDriveController>(
          // translation needs tuning and such
          pathplanner::PIDConstants{3.2, 0.0, 0.3},  // Translation PID constants
          pathplanner::PIDConstants{1.0, 0.0, 0.0}   // Rotation PID constants
      );

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

  // Sim pose estimation
  frc::SwerveDrivePoseEstimator<4> _simPoseEstimator{
      _kinematics,
      _gyro.GetRotation2d(),
      {frc::SwerveModulePosition{0_m, _frontLeft.GetAngle()},
       frc::SwerveModulePosition{0_m, _frontRight.GetAngle()},
       frc::SwerveModulePosition{0_m, _backLeft.GetAngle()},
       frc::SwerveModulePosition{0_m, _backRight.GetAngle()}},
      frc::Pose2d()};

  // Joystick controller rate limiters
  double _tunedMaxJoystickAccel = MAX_JOYSTICK_ACCEL;
  double _tunedMaxAngularJoystickAccel = MAX_ANGULAR_JOYSTICK_ACCEL;
  frc::SlewRateLimiter<units::scalar> _xStickLimiter{_tunedMaxJoystickAccel / 1_s};
  frc::SlewRateLimiter<units::scalar> _yStickLimiter{_tunedMaxJoystickAccel / 1_s};
  frc::SlewRateLimiter<units::scalar> _rotStickLimiter{_tunedMaxAngularJoystickAccel / 1_s};

  // Sysid
  frc2::sysid::SysIdRoutine _sysIdRoutine{
      frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt, nullptr},
      frc2::sysid::Mechanism{
          [this](units::volt_t driveVoltage) {
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