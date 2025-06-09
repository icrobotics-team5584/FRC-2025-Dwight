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

frc2::CommandPtr YAlignWithTarget(SubVision::Side side) 
{
  static frc::Pose2d targetPose;
  static frc::Pose2d targetTagPose;
  return RunOnce([side] {
    int tagId = SubVision::GetInstance().GetClosestTag(SubDrivebase::GetInstance().GetPose());
    frc::SmartDashboard::PutNumber("Closest tag", tagId);
    frc::Pose2d tagPose = SubVision::GetInstance().GetAprilTagPose(tagId);
    frc::SmartDashboard::PutNumber("ROTATIONTAGPOSE", tagPose.Rotation().Degrees().value());
    auto yOffset = (side == SubVision::Side::Left) ? 0.12_m : -0.2_m;//0.16_m : -0.16_m;
    targetTagPose = SubVision::GetInstance().CalculateRelativePose(tagPose, 0_m, yOffset);
    targetPose = SubVision::GetInstance().CalculateRelativePose(targetTagPose,0.6_m,0_m);
    targetTagPose = frc::Pose2d(targetTagPose.Translation(), targetTagPose.Rotation() + frc::Rotation2d(90_deg));
    targetPose = frc::Pose2d(targetPose.Translation(), targetPose.Rotation() + frc::Rotation2d(90_deg));
    SubDrivebase::GetInstance().DisplayPose("Vision 3d align pose", targetPose);
    SubDrivebase::GetInstance().DisplayPose("Vision 3d align target pose", targetTagPose);
  }).AndThen(
  SubDrivebase::GetInstance()
      .Drive(
          [side] {
            return SubDrivebase::GetInstance().CalcDriveToPoseSpeeds(targetPose) * 2.5;
          }, true)
      .Until([] {
        return SubDrivebase::GetInstance().IsAtPose(targetPose);
      })).AndThen(
  SubDrivebase::GetInstance()
      .Drive(
          [side] {
            return SubDrivebase::GetInstance().CalcDriveToPoseSpeeds(targetTagPose) * 0.3;
          }, true)
      .Until([] {
        return SubDrivebase::GetInstance().IsAtPose(targetTagPose);
}));
}

frc2::CommandPtr AddVisionMeasurement() {
  return Run(
      [] {
        auto estimatePoses = SubVision::GetInstance().GetPose();

        auto leftPose = estimatePoses[SubVision::Left];
        if (leftPose.has_value()) {
          if (SubVision::GetInstance().IsEstimateUsable(leftPose.value())) {
            auto estimatedPose = leftPose.value();
            double d = SubVision::GetInstance().GetDev(estimatedPose);
            wpi::array<double,3> dev = {d, d, 0.9};
            SubDrivebase::GetInstance().AddVisionMeasurement(
                estimatedPose.estimatedPose.ToPose2d(), estimatedPose.timestamp, dev);
            SubDrivebase::GetInstance().DisplayPose("LeftEstimatedPose",
                                                    estimatedPose.estimatedPose.ToPose2d());
          } else {
            SubDrivebase::GetInstance().DisplayPose("DiscardedLeftEstimatedPose", {leftPose.value().estimatedPose.ToPose2d()});
          }
        } else {
          SubDrivebase::GetInstance().DisplayPose("LeftEstimatedPose", {});
          SubDrivebase::GetInstance().DisplayPose("DiscardedLeftEstimatedPose", {});
        }

        auto rightPose = estimatePoses[SubVision::Right];
        if (rightPose.has_value()){
          if(SubVision::GetInstance().IsEstimateUsable(rightPose.value())) {
            auto estimatedPose = rightPose.value();
            double d = SubVision::GetInstance().GetDev(estimatedPose);
            wpi::array<double,3> dev = {d, d, 0.9};
            SubDrivebase::GetInstance().AddVisionMeasurement(
                estimatedPose.estimatedPose.ToPose2d(), estimatedPose.timestamp, dev);
            SubDrivebase::GetInstance().DisplayPose("RightEstimatedPose",
                                                    estimatedPose.estimatedPose.ToPose2d());
            } else {
              SubDrivebase::GetInstance().DisplayPose("DiscardedRightEstimatedPose", {rightPose.value().estimatedPose.ToPose2d()});
            }
        } else {
          SubDrivebase::GetInstance().DisplayPose("RightEstimatedPose", {});
          SubDrivebase::GetInstance().DisplayPose("DiscardedRightEstimatedPose", {});
        }
      
      },
      {&SubVision::GetInstance()}).IgnoringDisable(true);
} 

frc2::CommandPtr AlignAndShoot(SubVision::Side side)
{
  static frc::Pose2d targetPose;
  static frc::Pose2d targetTagPose;
  static frc::Pose2d awayPose;
  return RunOnce([side] {
    // int tagId = SubVision::GetInstance().GetClosestTag(SubDrivebase::GetInstance().GetPose());
    // frc::SmartDashboard::PutNumber("Closest tag", tagId);
    // frc::Pose2d tagPose = SubVision::GetInstance().GetAprilTagPose(tagId);
    // auto yOffset = (side == SubVision::Side::Left) ? 0.12_m : -0.2_m;//0.16_m : -0.16_m;
    // targetTagPose = SubVision::GetInstance().CalculateRelativePose(tagPose, 0_m, yOffset);
    targetTagPose = SubVision::GetInstance().GetReefPose(-1, side);
    targetPose = SubVision::GetInstance().CalculateRelativePose(targetTagPose,0_m,-0.4_m);
    targetTagPose = SubVision::GetInstance().CalculateRelativePose(targetTagPose,0_m,0_m);
    // targetTagPose = frc::Pose2d(targetTagPose.Translation(), targetTagPose.Rotation());
    // targetPose = frc::Pose2d(targetPose.Translation(), targetPose.Rotation());
    awayPose = SubVision::GetInstance().CalculateRelativePose(targetPose,0.0_m,0_m);
    SubDrivebase::GetInstance().DisplayPose("Vision 3d align pose", targetPose);
    SubDrivebase::GetInstance().DisplayPose("Vision 3d align target pose", targetTagPose);
  }).AndThen(
  SubDrivebase::GetInstance()
      .Drive(
          [side] {
            return SubDrivebase::GetInstance().CalcDriveToPoseSpeeds(targetPose) * 3.0;
          }, true)
      .Until([] {
        return SubDrivebase::GetInstance().IsAtPose(targetPose);
      })).AndThen(SubElevator::GetInstance().CmdSetElevatorToL()).AndThen(
  SubDrivebase::GetInstance()
      .Drive(
          [side] {
            return SubDrivebase::GetInstance().CalcDriveToPoseSpeeds(targetTagPose) * 0.7;
          }, true)
      .Until([] {
        return SubDrivebase::GetInstance().IsAtPose(targetTagPose) && SubElevator::GetInstance().IsAtTarget();
})).AndThen(SubEndEffector::GetInstance().ScoreCoral())
.Until([] {return !SubEndEffector::GetInstance().CheckLineBreakLower() && !SubEndEffector::GetInstance().CheckLineBreakHigher();})
.AndThen(
  SubDrivebase::GetInstance()
      .Drive(
          [side] { 
            return SubDrivebase::GetInstance().CalcDriveToPoseSpeeds(awayPose) * 1.3;
          }, true)
      .Until([] {
        return SubDrivebase::GetInstance().IsAtPose(awayPose);
})).AndThen(SubElevator::GetInstance().CmdSetSource());
}

}  // namespace cmd