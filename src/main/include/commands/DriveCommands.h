// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <units/length.h>
#include <frc2/command/button/CommandXboxController.h>

namespace cmd {
/**
 * Toggles brake and coast for the drivebase, climber motor and elevator motors.
 */
frc2::CommandPtr ToggleBrakeCoast();
frc2::CommandPtr TeleopDrive();
}  // namespace cmd