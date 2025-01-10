// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"
#include <frc2/command/Commands.h>
// auto includes
#include <pathplanner/lib/commands/PathPlannerAuto.h>

RobotContainer::RobotContainer() {
  SubVision::GetInstance();

  // Default Commands
  SubDrivebase::GetInstance().SetDefaultCommand(SubDrivebase::GetInstance().JoystickDrive(_driverController));
  
  // Trigger Bindings
  ConfigureBindings();

  // AutoChooser options
  _autoChooser.AddOption("Default-Left", "Default-Left");  
  _autoChooser.AddOption("Default-Middle", "placeholder-DM");
  _autoChooser.AddOption("Default-Right", "Default-Right");
  _autoChooser.AddOption("TeammateHelper-Left", "placeholder-THL");
  _autoChooser.AddOption("TeammateHelper-Right", "placeholder-THR");
  frc::SmartDashboard::PutData("Chosen Auton", &_autoChooser);   
}

void RobotContainer::ConfigureBindings() {
  _tuningController.A().WhileTrue(SubDrivebase::GetInstance().WheelCharecterisationCmd());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  //return pathplanner::PathPlannerAuto("test auto").ToPtr();
  auto _autoSelected = _autoChooser.GetSelected();
  units::second_t delay = 0.00_s;
  
  return frc2::cmd::Wait(delay)
    .AndThen(pathplanner::PathPlannerAuto(_autoSelected).ToPtr());
}
