#pragma once

#include <frc2/command/SubsystemBase.h>
#include <studica/AHRS.h>
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
#include "utilities/BotVars.h"
#include "utilities/RobotLogs.h"

class SubDrivebase : public frc2::SubsystemBase {
 public:
  SubDrivebase();
  static SubDrivebase &GetInstance() {
    static SubDrivebase inst;
    return inst;
  }
  void Periodic() override;
  void SimulationPeriodic() override;

  // Instantaneous functions
  void AddVisionMeasurement(frc::Pose2d pose, double ambiguity, units::second_t timeStamp);
  void ResetGyroHeading(units::degree_t startingAngle = 0_deg);
  void DisplayTrajectory(std::string name, frc::Trajectory trajectory);
  void SetNeutralMode(bool mode);
  void SetPose(frc::Pose2d pose);
  void DisplayPose(std::string label, frc::Pose2d pose);
  void UpdateOdometry();
  void SyncSensors();
  void SetPathplannerRotationFeedbackSource(std::function<units::turns_per_second_t()> rotationFeedbackSource);
  void ResetPathplannerRotationFeedbackSource();

  // Getters
  bool IsAtPose(frc::Pose2d pose);
  frc::ChassisSpeeds CalcDriveToPoseSpeeds(frc::Pose2d targetPose);
  frc::ChassisSpeeds CalcJoystickSpeeds(frc2::CommandXboxController& controller);
  frc::ChassisSpeeds CalcJoystickSpeedsEndEffectorForward(frc2::CommandXboxController& controller);
  units::turns_per_second_t CalcRotateSpeed(units::turn_t rotationError);
  units::degree_t GetPitch();
  frc::Pose2d GetPose();
  frc::Pose2d GetSimPose();
  frc::Rotation2d GetHeading(); // Heading as recorded by the pose estimator (matches field orientation)
  frc::Rotation2d GetGyroAngle(); // Heading as recorded by the gyro (zero is direction when switched on)
  units::meters_per_second_t GetVelocity();
  frc::SwerveDriveKinematics<4> GetKinematics();
  frc::ChassisSpeeds GetRobotRelativeSpeeds();
  frc2::CommandPtr WheelCharecterisationCmd();

  // Commands
  frc2::CommandPtr JoystickDrive(frc2::CommandXboxController& controller);
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
  static constexpr units::turns_per_second_t MAX_ANGULAR_VELOCITY = 720_deg_per_s;
  static constexpr units::turns_per_second_squared_t MAX_ANG_ACCEL{std::numbers::pi};
  static constexpr double MAX_JOYSTICK_ACCEL = 3;
  static constexpr double MAX_ANGULAR_JOYSTICK_ACCEL = 3;
  static constexpr double JOYSTICK_DEADBAND = 0.08;

 private:
  void Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed,
             units::turns_per_second_t rot, bool fieldRelative,
             std::optional<std::array<units::newton_t, 4>> xForceFeedforwards = std::nullopt,
             std::optional<std::array<units::newton_t, 4>> yForceFeedforwards = std::nullopt);

  studica::AHRS _gyro{studica::AHRS::NavXComType::kUSB1};

  // Swerve modules
  frc::Translation2d _frontLeftLocation{+0.281_m, +0.281_m};
  frc::Translation2d _frontRightLocation{+0.281_m, -0.281_m};
  frc::Translation2d _backLeftLocation{-0.281_m, +0.281_m};
  frc::Translation2d _backRightLocation{-0.281_m, -0.281_m};

  // const units::turn_t FRONT_RIGHT_MAG_OFFSET = -0.375732_tr;
  // const units::turn_t FRONT_LEFT_MAG_OFFSET = -0.941406_tr;
  // const units::turn_t BACK_RIGHT_MAG_OFFSET = -0.462891_tr;
  // const units::turn_t BACK_LEFT_MAG_OFFSET = -0.329590_tr;

  const double FRONT_RIGHT_MAG_OFFSET = BotVars::Choose(-0.876953125 + 0.5, -0.515380859375);
  const double FRONT_LEFT_MAG_OFFSET = BotVars::Choose(-0.443603515625+ 0.5, -0.172607421875);
  const double BACK_RIGHT_MAG_OFFSET = BotVars::Choose(-0.962158203125+ 0.5, -0.395263671875);
  const double BACK_LEFT_MAG_OFFSET = BotVars::Choose(-0.840087890625+ 0.5, -0.94921875);

  SwerveModule _frontLeft{canid::DriveBaseFrontLeftDrive, canid::DriveBaseFrontLeftTurn,
                          canid::DriveBaseFrontLeftEncoder, (FRONT_LEFT_MAG_OFFSET*1_tr) };
  SwerveModule _frontRight{canid::DriveBaseFrontRightDrive, canid::DriveBaseFrontRightTurn,
                           canid::DriveBaseFrontRightEncoder, (FRONT_RIGHT_MAG_OFFSET*1_tr) };
  SwerveModule _backLeft{canid::DriveBaseBackLeftDrive, canid::DriveBaseBackLeftTurn,
                         canid::DriveBaseBackLeftEncoder, (BACK_LEFT_MAG_OFFSET*1_tr) };
  SwerveModule _backRight{canid::DriveBaseBackRightDrive, canid::DriveBaseBackRightTurn,
                          canid::DriveBaseBackRightEncoder, (BACK_RIGHT_MAG_OFFSET*1_tr) };

  // Control objects
  frc::SwerveDriveKinematics<4> _kinematics{_frontLeftLocation, _frontRightLocation,
                                            _backLeftLocation, _backRightLocation};

  frc::PIDController _teleopTranslationController{Logger::Tune("tuner/TeleopP", 5.0), Logger::Tune("tuner/TeleopI", 0.0), Logger::Tune("tuner/TeleopD", 50.0)};
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
