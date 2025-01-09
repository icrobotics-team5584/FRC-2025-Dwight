// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"
#include "commands/VisionCommand.h"
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  SubVision::GetInstance();

  // Default Commands
  SubDrivebase::GetInstance().SetDefaultCommand(SubDrivebase::GetInstance().JoystickDrive(_driverController));
  
  // Trigger Bindings
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  // _driverController.A().WhileTrue(SubDrivebase::GetInstance().WheelCharecterisationCmd());
  _driverController.X().WhileTrue(cmd::YAlignWithTarget(0.165_m));
  _driverController.Y().WhileTrue(cmd::YAlignWithTarget(-0.165_m));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
