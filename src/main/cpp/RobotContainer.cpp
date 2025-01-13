// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/SubDrivebase.h"
#include <frc2/command/Commands.h>
// auto includes
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include "subsystems/SubDrivebase.h"

RobotContainer::RobotContainer() {

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
  _autoChooser.AddOption("L-Shape", "L-Shape");
  _autoChooser.AddOption("L-Shape-Slow", "L-Shape-Slow");
  _autoChooser.AddOption("L-Shape-Spinning", "L-Shape-Spinning");
  _autoChooser.AddOption("L-Shape-Spinning-Slow", "L-Shape-Spinning-Slow");
  _autoChooser.AddOption("move", "move");
  frc::SmartDashboard::PutData("Chosen Auton", &_autoChooser);   
}

void RobotContainer::ConfigureBindings() {
  _driverController.A().WhileTrue(SubDrivebase::GetInstance().WheelCharecterisationCmd());
  _driverController.Y().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());
  _driverController.X().WhileTrue(SubDrivebase::GetInstance().Drive([] {return frc::ChassisSpeeds(0.1_mps, 0_mps, 0.5_tps);} ,false));
  _driverController.B().WhileTrue(SubDrivebase::GetInstance().Drive([] {return frc::ChassisSpeeds(-0.1_mps, 0_mps, 0.5_tps);} ,false));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  //return pathplanner::PathPlannerAuto("test auto").ToPtr();
  auto _autoSelected = _autoChooser.GetSelected();
  units::second_t delay = 0.00_s;
  
  return frc2::cmd::Wait(delay)
    .AndThen(pathplanner::PathPlannerAuto(_autoSelected).ToPtr());
}

// Controller rumble functions
frc2::CommandPtr RobotContainer::ControllerRumbleLeft(frc2::CommandXboxController& controller) {
  return frc2::cmd::StartEnd(
      [this, &controller] { controller.SetRumble(frc::XboxController::RumbleType::kRightRumble, 1.0); },
      [this, &controller] { controller.SetRumble(frc::XboxController::RumbleType::kRightRumble, 0); });
}

frc2::CommandPtr RobotContainer::ControllerRumbleRight(frc2::CommandXboxController& controller) {
  return frc2::cmd::StartEnd(
      [this, &controller] { controller.SetRumble(frc::XboxController::RumbleType::kLeftRumble, 1.0); },
      [this, &controller] { controller.SetRumble(frc::XboxController::RumbleType::kLeftRumble, 0); });
}


