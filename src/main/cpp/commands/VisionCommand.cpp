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

frc2::CommandPtr FrontApproachAlign (frc::Pose2d targetPose) {
  return SubDrivebase::GetInstance().Drive([targetPose] {
    auto RelativeDifference = SubDrivebase::GetInstance().GetPose().RelativeTo(targetPose);
    auto RotationDifference = SubDrivebase::GetInstance().GetGyroAngle().Degrees() + 90_deg - targetPose.Rotation().Degrees();
    frc::SmartDashboard::PutNumber("Vision/3D align X difference", RelativeDifference.X().value());
    frc::SmartDashboard::PutNumber("Vision/3D align Y difference", RelativeDifference.Y().value());
    frc::SmartDashboard::PutNumber("Vision/3D align roation difference",RotationDifference.value());
    frc::SmartDashboard::PutNumber("Vision/3D align Z difference", RelativeDifference.Rotation().Degrees().value());
    // frc::ChassisSpeeds speeds {0_mps, 0.5_mps, 0_tps};
    frc::ChassisSpeeds speeds {0_mps, Logger::Tune("visionalignforwardspeed", 0.7)*1_mps, 0_tps};
    // if (RotationDifference < -2_deg) {
    //   speeds.omega = -RotationDifference.value()*0.5_deg_per_s;
    // } else if (RotationDifference > 2_deg) {
    //   speeds.omega = RotationDifference.value()*0.5_deg_per_s;
    // }
    // if (RelativeDifference.Y() < -0.01_m) {
    //   speeds.vx = -0.1_mps;
    // } else if (RelativeDifference.Y() > 0.01_m) {
    //   speeds.vx = 0.1_mps;
    // }
    return speeds;
  }, false);
}

frc2::CommandPtr ForceAlignWithTarget(SubVision::Side side) {
  // Strafe until the tag is at a known scoring angle, with a small velocity component towards the
  // reef so you stay aligned rotationally and at the right distance.
  return SubDrivebase::GetInstance().Drive(
      [side] {
        if (SubVision::GetInstance().GetLastReefTagArea() > 0) { // 0 is for removing the move forward 
          units::degree_t goalAngle;
          units::degree_t tagAngle = SubVision::GetInstance().GetLastReefTagAngle();
          
          if (Logger::Tune("Vision/use dashbaord target", false)) {
            goalAngle =
                Logger::Tune("Vision/Goal Angle", SubVision::GetInstance().GetReefAlignAngle(side));
          } else {
            goalAngle = SubVision::GetInstance().GetReefAlignAngle(
                side);  // 16.45 for left reef face, 15.60_deg for front reef face (all left side so
                        // far) // 15.60,-20
          }
          units::degree_t error = tagAngle - goalAngle;
          
          units::meters_per_second_t overallVelocity = std::clamp(0.16_mps * error.value(),-0.3_mps,0.3_mps);
          units::degree_t driveAngle = units::math::copysign(8_deg, error);
          
          // Calc x and y from known angle and hypotenuse length
          units::meters_per_second_t xVel = overallVelocity * units::math::cos(driveAngle);
          units::meters_per_second_t yVel = overallVelocity * units::math::sin(driveAngle);
          
          Logger::Log("Vision/Alignment Overall Velocity", overallVelocity);
          Logger::Log("Vision/Tag Angle", tagAngle);
          Logger::Log("Vision/Goal Angle", goalAngle);
          Logger::Log("Vision/Target Pole", side == SubVision::Side::Left ? "Left":"Right");
          
          // rotation componet
          frc::Rotation2d targetRotation = SubVision::GetInstance().GetReefPose(-1,side).Rotation();

          units::angle::turn_t roterror = SubDrivebase::GetInstance().GetPose().Rotation().Degrees() - targetRotation.Degrees();
          auto rotationSpeed = SubDrivebase::GetInstance().CalcRotateSpeed(roterror) / 5;

          return frc::ChassisSpeeds{xVel, yVel, rotationSpeed};
          
        } else {
          frc::Rotation2d targetRotation = SubVision::GetInstance().GetLastReefPose(side).Rotation();
          units::angle::turn_t roterror = SubDrivebase::GetInstance().GetAllianceRelativeGyroAngle().Degrees() - targetRotation.Degrees();
          auto rotationSpeed = SubDrivebase::GetInstance().CalcRotateSpeed(roterror) / 5;
          return frc::ChassisSpeeds{0_mps, 0.5_mps, rotationSpeed};
        }
      },
      false);
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

// check pose -> decide which source is closer -> drive there
frc2::CommandPtr AlignToSource() {
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

frc2::CommandPtr AlignAndShoot(SubVision::Side side)
{
  static frc::Pose2d targetPose;
  static frc::Pose2d targetTagPose;
  static frc::Pose2d awayPose;
  return RunOnce([side] {
    int tagId = SubVision::GetInstance().GetClosestTag(SubDrivebase::GetInstance().GetPose());
    frc::SmartDashboard::PutNumber("Closest tag", tagId);
    frc::Pose2d tagPose = SubVision::GetInstance().GetAprilTagPose(tagId);
    frc::SmartDashboard::PutNumber("ROTATIONTAGPOSE", tagPose.Rotation().Degrees().value());
    auto yOffset = (side == SubVision::Side::Left) ? 0.12_m : -0.2_m;//0.16_m : -0.16_m;
    targetTagPose = SubVision::GetInstance().CalculateRelativePose(tagPose, 0_m, yOffset);
    targetPose = SubVision::GetInstance().CalculateRelativePose(targetTagPose,0.6_m,0_m);
    targetTagPose = SubVision::GetInstance().CalculateRelativePose(targetTagPose,0.45_m,0_m);
    targetTagPose = frc::Pose2d(targetTagPose.Translation(), targetTagPose.Rotation() + frc::Rotation2d(90_deg));
    targetPose = frc::Pose2d(targetPose.Translation(), targetPose.Rotation() + frc::Rotation2d(90_deg));
    awayPose = SubVision::GetInstance().CalculateRelativePose(targetPose,0.0_m,-0.2_m);
    SubDrivebase::GetInstance().DisplayPose("Vision 3d align pose", targetPose);
    SubDrivebase::GetInstance().DisplayPose("Vision 3d align target pose", targetTagPose);
  }).AndThen(
  SubDrivebase::GetInstance()
      .Drive(
          [side] {
            return SubDrivebase::GetInstance().CalcDriveToPoseSpeeds(targetPose) * 3;
          }, true)
      .Until([] {
        return SubDrivebase::GetInstance().IsAtPose(targetPose);
      })).AndThen(SubElevator::GetInstance().CmdSetElevatorToL()).AndThen(
  SubDrivebase::GetInstance()
      .Drive(
          [side] {
            return SubDrivebase::GetInstance().CalcDriveToPoseSpeeds(targetTagPose) * 0.4;
          }, true)
      .Until([] {
        return SubDrivebase::GetInstance().IsAtPose(targetTagPose) && SubElevator::GetInstance().IsAtTarget();
})).AndThen(SubEndEffector::GetInstance().ScoreCoral())
.Until([] {return !SubEndEffector::GetInstance().CheckLineBreakLower() && !SubEndEffector::GetInstance().CheckLineBreakHigher();})
.AndThen(
  SubDrivebase::GetInstance()
      .Drive(
          [side] { 
            return SubDrivebase::GetInstance().CalcDriveToPoseSpeeds(awayPose) * 0.8;
          }, true)
      .Until([] {
        return SubDrivebase::GetInstance().IsAtPose(awayPose);
})).AndThen(SubElevator::GetInstance().CmdSetSource());
}
frc2::CommandPtr HopeAndShoot(SubVision::Side side) {
  return ForceAlignWithTarget(side).AlongWith(AutoShootIfKindaAligned(side));
}

// frc2::CommandPtr AutoShootIfAligned(SubVision::Side side) {
//   return Sequence(
//     WaitUntil([side] {
//       units::degree_t goalAngle = SubVision::GetInstance().GetReefAlignAngle(side);
//       units::degree_t tagAngle = SubVision::GetInstance().GetLastReefTagAngle();
//       double visionTolerance = Logger::Tune("AutoShoot/AutoShootVisionTolerance", 3.0);
//       double drivebaseTolerance = Logger::Tune("AutoShoot/AutoShootDrivebaseTolerance", 0.01);
//       Logger::Log("AutoShoot/errorangle", (goalAngle-tagAngle).value());
     
//       bool inTagAngleRange = false; 
//       bool isAtTargetHeight = false;
//       bool isStill = false;

//       // vision tag angles 
//       if (tagAngle < goalAngle + visionTolerance*1_deg && tagAngle > goalAngle - visionTolerance*1_deg) {
//         inTagAngleRange = true;
//       }

//       // height target 
//       if (SubElevator::GetInstance().GetTargetHeight() != SubElevator::_SOURCE_HEIGHT && SubElevator::GetInstance().IsAtTarget()){
//         isAtTargetHeight =  SubElevator::GetInstance().IsAtTarget();
//       }

//       // is still
//       if (SubDrivebase::GetInstance().GetVelocity() < drivebaseTolerance *1_mps) {
//         isStill = true;
//       }
      
//       if (isAtTargetHeight) {  // && inTagAngleRange
//         return true;
//       }
//       return false;
//       }),
//     frc2::cmd::WaitUntil(
//       [] { return SubElevator::GetInstance().IsAtTarget(); }),
//     frc2::cmd::Wait(0.3_s),
//     SubEndEffector::GetInstance().ScoreCoral()
//   );
// }

frc2::CommandPtr AutoShootIfKindaAligned(SubVision::Side side) {
  return Sequence(
    WaitUntil([side] {
      units::degree_t goalAngle = SubVision::GetInstance().GetReefAlignAngle(side);
      units::degree_t tagAngle = SubVision::GetInstance().GetLastReefTagAngle();
      double visionTolerance = Logger::Tune("AutoShoot/AutoShootVisionLooseTolerance", 1.0);
      double drivebaseTolerance = Logger::Tune("AutoShoot/AutoShootDrivebaseLooseTolerance", 0.01);
      Logger::Log("AutoShoot/errorangle", (goalAngle-tagAngle).value());
     
      bool inTagAngleRange = false; 
      bool isAtTargetHeight = false;
      bool isStill = false;

      // vision tag angles 
      if (tagAngle < goalAngle + visionTolerance*1_deg && tagAngle > goalAngle - visionTolerance*1_deg) {
        inTagAngleRange = true;
      }

      // height target 
      if (SubElevator::GetInstance().GetTargetHeight() != SubElevator::_SOURCE_HEIGHT){
        isAtTargetHeight =  SubElevator::GetInstance().IsAtTarget();
      }

      // is still
      if (SubDrivebase::GetInstance().GetVelocity() < drivebaseTolerance *1_mps) {
        isStill = true;
      }
      
      if (isStill && isAtTargetHeight && inTagAngleRange) {
        return true;
      }
      return false;
      }),
    SubEndEffector::GetInstance().ScoreCoral()
  );
}


}  // namespace cmd