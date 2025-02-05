// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveCommands.h"
#include <frc2/command/Commands.h>
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"
#include "subsystems/SubClimber.h"
#include "subsystems/SubElevator.h"
#include <frc/DriverStation.h>

namespace cmd {
using namespace frc2::cmd;
/**
 * Command to align to Apriltag with y offset (to match left or right of reef)
 *
 * @param offset The y offset from the tag to align to, in meters.
 * @return A command that will align to the tag when executed.
 */

frc2::CommandPtr ToggleBrakeCoast() {
  return StartEnd(
             [] {
               SubDrivebase::GetInstance().SetBrakeMode(false);
               SubClimber::GetInstance().SetBrakeMode(false);
               SubElevator::GetInstance().SetBrakeMode(false);
             },
             [] {
               SubDrivebase::GetInstance().SetBrakeMode(true);
               SubClimber::GetInstance().SetBrakeMode(true);
               SubElevator::GetInstance().SetBrakeMode(true);
             })
      .IgnoringDisable(true)
      .Until([] { return frc::DriverStation::IsEnabled(); });
}
}  // namespace cmd
