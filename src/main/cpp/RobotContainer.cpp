// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/SubElevator.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer() {
  ConfigureBindings();
};


void RobotContainer::ConfigureBindings() {
  
  //Driver

  //Triggers

  //Bumpers

  //Letters
   _operatorController.A().OnTrue(SubElevator::GetInstance().CmdSetL1());
   _operatorController.B().OnTrue(SubElevator::GetInstance().CmdSetL2());
   _operatorController.X().OnTrue(SubElevator::GetInstance().CmdSetL3());
   _operatorController.Y().OnTrue(SubElevator::GetInstance().CmdSetL4());

  //POV

  //Operator

  //Triggers

  
  //Bumpers


  //Letters


//POV

};


frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print(".");
}
