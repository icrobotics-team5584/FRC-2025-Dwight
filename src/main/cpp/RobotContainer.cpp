// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include "subsystems/SubIntake.h"
#include "subsystems/SubEndEffector.h"

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  _controller.A().WhileTrue(SubIntake::GetInstance().Intake());
  _controller.B().WhileTrue(SubIntake::GetInstance().Outtake());
  _controller.X().WhileTrue(SubIntake::GetInstance().Deploy());
  _controller.Y().WhileTrue(SubIntake::GetInstance().Stow());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
