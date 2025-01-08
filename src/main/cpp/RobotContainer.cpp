// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  SubDrivebase::GetInstance().SetDefaultCommand(SubDrivebase::GetInstance().JoystickDrive(_driverController));
  SubVision::GetInstance();
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
