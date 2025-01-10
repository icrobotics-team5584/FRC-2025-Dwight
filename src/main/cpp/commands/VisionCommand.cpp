// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/VisionCommand.h"
#include "subsystems/SubVision.h"
#include "subsystems/SubDriveBase.h"
#include <frc2/command/Commands.h>
#include <frc/DriverStation.h>
namespace cmd {
using namespace frc2::cmd;
/**
 * Command to align to Apriltag with y offset (to match left or right of reef)
 *
 * @param offset The y offset from the tag to align to, in meters.
 * @return A command that will align to the tag when executed.
 */ 

frc2::CommandPtr YAlignWithTarget(units::meter_t offset, frc2::CommandXboxController &controller) {
  return SubDrivebase::GetInstance().Drive([offset, &controller] {
    frc::Pose2d tagPose = SubVision::GetInstance().GetBestTarget();
    frc::Translation2d robotPose = SubDrivebase::GetInstance().GetPose().Translation();
    frc::Translation2d relatedRobotTranslation = robotPose - tagPose.Translation();
    frc::Translation2d netRobotTrans = relatedRobotTranslation.RotateBy(-tagPose.Rotation());
    frc::Translation2d adjRelatedBotTrans =
        frc::Translation2d(netRobotTrans.X(), offset).RotateBy(tagPose.Rotation());

    frc::Pose2d targetPose =
        frc::Pose2d(adjRelatedBotTrans + tagPose.Translation(), tagPose.Rotation() - 180_deg);
    frc::ChassisSpeeds speeds = SubDrivebase::GetInstance().CalcDriveToPoseSpeeds(targetPose);

    auto joystickSpeeds = SubDrivebase::GetInstance().CalcJoystickSpeeds(controller);
    speeds.vx = joystickSpeeds.vx;

    return speeds;
  }, true);
}


frc2::CommandPtr AddVisionMeasurement(){
  return Run(
      [] {
        auto estimatedPose = SubVision::GetInstance().GetPose();
        frc::SmartDashboard::PutBoolean("Vision/is teleop", frc::DriverStation::IsTeleop());
        frc::SmartDashboard::PutBoolean("Vision/has value", estimatedPose.has_value());
        if (estimatedPose.has_value() && frc::DriverStation::IsTeleop()) {
          auto estimatedPoseValue = estimatedPose.value();
          SubDrivebase::GetInstance().AddVisionMeasurement(
              estimatedPoseValue.estimatedPose.ToPose2d(), 0,
              estimatedPoseValue.timestamp);
          SubDrivebase::GetInstance().DisplayPose(
              "EstimatedPose", estimatedPoseValue.estimatedPose.ToPose2d());
        } else {
          SubDrivebase::GetInstance().DisplayPose("EstimatedPose", {});
        }
      },
      {&SubVision::GetInstance()});
}  // namespace cmd

}