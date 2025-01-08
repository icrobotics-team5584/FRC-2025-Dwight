// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/Commands.h>
#include "subsystems/SubIntake.h"
#include "subsystems/SubEndEffector.h"
#include "commands/CoralCommands.h"

RobotContainer::RobotContainer() {
  ConfigureBindings();
}

void RobotContainer::ConfigureBindings() {
  SubIntake& intake = SubIntake::GetInstance();

  _controller.A().WhileTrue(cmd::IntakeFullSequence());
  _controller.A().OnFalse(SubEndEffector::GetInstance().StopMotor().AlongWith(intake.Stow()));
  _controller.B().WhileTrue(intake.Outtake());
  _controller.X().WhileTrue(intake.Deploy());
  _controller.Y().WhileTrue(intake.Stow());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
