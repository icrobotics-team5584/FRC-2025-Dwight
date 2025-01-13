// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/SubDrivebase.h"
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
  SubDrivebase::GetInstance().SetDefaultCommand(SubDrivebase::GetInstance().JoystickDrive(_driverController));
}

void RobotContainer::ConfigureBindings() {
  _driverController.X().WhileTrue(SubDrivebase::GetInstance().Drive([] {return frc::ChassisSpeeds(0.5_mps, 0_mps, 0.2_tps);}, true));
  _driverController.Y().WhileTrue(SubDrivebase::GetInstance().Drive([] {return frc::ChassisSpeeds(-0.5_mps, 0_mps, -0.2_tps);}, true));
  _driverController.B().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
