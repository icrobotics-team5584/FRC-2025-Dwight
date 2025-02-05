// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveCommands.h"
#include <frc2/command/Commands.h>
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"



namespace cmd {
using namespace frc2::cmd;
/**
 * Command to align to Apriltag with y offset (to match left or right of reef)
 *
 * @param offset The y offset from the tag to align to, in meters.
 * @return A command that will align to the tag when executed.
 */

/*
frc2::CommandPtr AlignToTarget(units::meter_t offset) {
  return Run([] {
    SubDrivebase::GetInstance().DriveToPose(
        frc::Pose2d(frc::Translation2d(0_m, 0_m), frc::Rotation2d(0_deg)));
  });
}

*/
  frc2::CommandPtr toggleBrakeCoast() {
    return StartEnd([] { SubDrivebase::GetInstance().SetNeutralMode(false); },
                    [] { SubDrivebase::GetInstance().SetNeutralMode(true); })
        .IgnoringDisable(true);
  } 
}  // namespace cmd
