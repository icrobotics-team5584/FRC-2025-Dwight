// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/VisionCommand.h"
#include "subsystems/SubVision.h"
#include "subsystems/SubDrivebase.h"
#include <frc2/command/Commands.h>
#include <frc/DriverStation.h>
#include "subsystems/SubElevator.h"
#include "subsystems/SubEndEffector.h"
#include "utilities/RobotLogs.h"
#include "iostream"
namespace cmd {
using namespace frc2::cmd;
/**
 * Command to align to Apriltag with y offset (to match left or right of reef)
 * @param side align to left: 1, align to right: 2
 * @return A command that will align to the tag when executed.* 3.500_m, 3.870
 */
frc2::CommandPtr YAlignWithTarget(int side, frc2::CommandXboxController& controller) 
{
  static frc::Pose2d targetPose;
  return SubDrivebase::GetInstance()
      .Drive(
          [side, &controller] {
            targetPose = SubVision::GetInstance().GetReefPose(side);
            frc::ChassisSpeeds speeds =
                SubDrivebase::GetInstance().CalcDriveToPoseSpeeds(targetPose);

            return speeds;
          },
          true)
      .AlongWith(Run([]{ frc::SmartDashboard::PutNumber("Drivebase/targetpose", targetPose.X().value()); }))
      .Until([] {
        return SubDrivebase::GetInstance().IsAtPose(targetPose);
      })
      .AndThen(SubDrivebase::GetInstance().Drive(
          [] { return frc::ChassisSpeeds{0_mps, 0.15_mps, 0_deg_per_s}; }, false));
}

frc2::CommandPtr YAutonAlignWithTarget(int side) {
  
  static frc::Pose2d targetPose; //= SubVision::GetInstance().GetReefPose(side);
  // printf("\nhello-\nTargetPose-GetReefPose-ROTATION%f deg\n", targetPose.Rotation().Degrees().value());
  // printf("\nhello-\nTargetPose-GetReefPose-TRANS-X%f m\n", targetPose.Translation().X().value());
  // printf("\nhello-\nTargetPose-GetReefPose-TRANS-X%f m\n", targetPose.Translation().X().value());
  return SubDrivebase::GetInstance()
      .Drive(
          [side] {
            frc::Pose2d targetPose = SubVision::GetInstance().GetReefPose(side);
            frc::ChassisSpeeds speeds =
                SubDrivebase::GetInstance().CalcDriveToPoseSpeeds(targetPose);
                //printf("SPEEDS\nvelx:%f_m/s\nvely:%f_m/s\nomega:%f_rad/s\n\n",speeds.vx.value(), speeds.vy.value(), speeds.omega.value());
            return speeds;
          }, 
          true)
      .Until([] {
        return SubDrivebase::GetInstance().IsAtPose(frc::Pose2d{
            targetPose.X(), targetPose.Y(), SubDrivebase::GetInstance().GetPose().Rotation()});
            printf("I AM AT TARGET POSE");
      })
      .AndThen(SubDrivebase::GetInstance().Drive(
          [] { return frc::ChassisSpeeds{0_mps, 0.5_mps, 0_deg_per_s}; }, false));
}
  
frc2::CommandPtr ForceAlignWithTarget(int side, frc2::CommandXboxController& controller) {
  // Strafe until the tag is at a known scoring angle, with a small velocity component towards the
  // reef so you stay aligned rotationally and at the right distance.
  return SubDrivebase::GetInstance()
    .Drive(
      [side, &controller] {
        if (SubVision::GetInstance().GetReefArea() > 3.5) {
          units::degree_t goalAngle;
          if (Logger::Tune("Vision/use dashbaord target", false)) {
            goalAngle = Logger::Tune("Vision/Goal Angle", SubVision::GetInstance().GetReefAlignAngle(1));
          } else {
            goalAngle = SubVision::GetInstance().GetReefAlignAngle(side); // 16.45 for left reef face, 15.60_deg for front reef face (all left side so far) // 15.60,-20
          }
          units::degree_t tagAngle = SubVision::GetInstance().GetReefTagAngle();
          units::degree_t error = tagAngle - goalAngle;
          units::meters_per_second_t overallVelocity = std::clamp(0.12_mps * error.value(),-0.3_mps,0.3_mps);
          units::degree_t driveAngle = units::math::copysign(8_deg, error);
          // Calc x and y from known angle and hypotenuse length
          units::meters_per_second_t xVel = overallVelocity * units::math::cos(driveAngle);
          units::meters_per_second_t yVel = overallVelocity * units::math::sin(driveAngle);
          return frc::ChassisSpeeds{xVel, yVel, 0_deg_per_s};
        } else {
          frc::Rotation2d targetRotation = SubVision::GetInstance().GetReefPose(side).Rotation();
          units::angle::turn_t roterror = SubDrivebase::GetInstance().GetPose().Rotation().Degrees() - targetRotation.Degrees();
          auto rotationSpeed = SubDrivebase::GetInstance().CalcRotateSpeed(roterror) / 5;
          return frc::ChassisSpeeds{0_mps, 0.5_mps, rotationSpeed};
        }
      },
      false);
}

frc2::CommandPtr AddVisionMeasurement() {
  return Run(
      [] {
        auto estimatedPose = SubVision::GetInstance().GetPose();
        frc::SmartDashboard::PutBoolean("Vision/is teleop", frc::DriverStation::IsTeleop());
        frc::SmartDashboard::PutBoolean("Vision/has value", estimatedPose.has_value());
        if (estimatedPose.has_value() && frc::DriverStation::IsTeleop()) {
          auto estimatedPoseValue = estimatedPose.value();
          SubDrivebase::GetInstance().AddVisionMeasurement(
              estimatedPoseValue.estimatedPose.ToPose2d(), 0, estimatedPoseValue.timestamp);
          SubDrivebase::GetInstance().DisplayPose("EstimatedPose",
                                                  estimatedPoseValue.estimatedPose.ToPose2d());
        } else {
          SubDrivebase::GetInstance().DisplayPose("EstimatedPose", {});
        }
      },
      {&SubVision::GetInstance()});
}  // namespace cmd
// check pose -> decide which source is closer -> drive there
frc2::CommandPtr AlignToSource(frc2::CommandXboxController& controller) {
  return SubDrivebase::GetInstance().Drive(
      [&controller] {
        frc::Pose2d sourcePose = {0_m, 0_m, 0_deg};
        units::meter_t poseY = SubDrivebase::GetInstance().GetPose().Y();
        if (poseY > 4.025_m) {
          if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) {
            sourcePose = SubVision::GetInstance().GetSourcePose(2);
          } else {
            sourcePose = SubVision::GetInstance().GetSourcePose(13);
          }
        } else {
          if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) {
            sourcePose = SubVision::GetInstance().GetSourcePose(1);
          } else {
            sourcePose = SubVision::GetInstance().GetSourcePose(12);
          }
        }
        return SubDrivebase::GetInstance().CalcDriveToPoseSpeeds(sourcePose);
      },
      true);
}
frc2::CommandPtr AutonAlignToSource() {
  return SubDrivebase::GetInstance().Drive(
      [] {
        frc::Pose2d sourcePose = {0_m, 0_m, 0_deg};
        units::meter_t poseY = SubDrivebase::GetInstance().GetPose().Y();
        if (poseY > 4.025_m) {
          if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) {
            sourcePose = SubVision::GetInstance().GetSourcePose(2);
          } else {
            sourcePose = SubVision::GetInstance().GetSourcePose(13);
          }
        } else {
          if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) {
            sourcePose = SubVision::GetInstance().GetSourcePose(1);
          } else {
            sourcePose = SubVision::GetInstance().GetSourcePose(12);
          }
        }
        return SubDrivebase::GetInstance().CalcDriveToPoseSpeeds(sourcePose);
      },
      true);
}

frc2::CommandPtr AutoShootIfAligned(int side) {
  return Sequence(
    WaitUntil([side] {return SubDrivebase::GetInstance().IsAtPose(SubVision::GetInstance().GetReefPose(side));}),
    WaitUntil([] {return SubElevator::GetInstance().IsAtTarget();}),
    SubEndEffector::GetInstance().ScoreCoral()
  );
}
}  // namespace cmd