// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/VisionCommand.h"
#include "subsystems/SubVision.h"
#include "subsystems/SubDriveBase.h"
#include <frc2/command/Commands.h>

namespace cmd {
using namespace frc2::cmd;
frc2::CommandPtr YAlignWithTarget(units::meter_t offset) {
  return Run([offset] {
    frc::Pose2d tagPose = SubVision::GetInstance().GetBestTarget();
    frc::Translation2d robotPose = SubDrivebase::GetInstance().GetPose().Translation();
    frc::Translation2d relatedRobotTranslation = robotPose - tagPose.Translation();
    frc::Translation2d netRobotTrans = relatedRobotTranslation.RotateBy(-tagPose.Rotation());
    frc::Translation2d adjRelatedBotTrans = frc::Translation2d(netRobotTrans.X(),offset).RotateBy(tagPose.Rotation());

    frc::Pose2d targetPose = frc::Pose2d(adjRelatedBotTrans + tagPose.Translation(), tagPose.Rotation() - 180_deg);
    SubDrivebase::GetInstance().DriveToPose(targetPose);
  });
}
}  // namespace cmd