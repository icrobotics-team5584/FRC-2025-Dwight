// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <units/length.h>
#include <frc2/command/button/CommandXboxController.h>



namespace cmd {
  /**
   * Command to align to Apriltag with y offset (to match left or right of reef)
   */
  frc2::CommandPtr YAlignWithTarget(int side, frc2::CommandXboxController &controller);
  frc2::CommandPtr AddVisionMeasurement();
  frc2::CommandPtr AlignToSource(frc2::CommandXboxController &controller);
  frc2::CommandPtr AutoShootIfAligned(int side);
  frc2::CommandPtr ForceAlignWithTarget(int side, frc2::CommandXboxController &controller);
}