// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"
#include <frc2/command/Commands.h>
<<<<<<< HEAD
#include <frc/smartdashboard/SmartDashboard.h>
=======
#include "subsystems/SubIntake.h"
#include "subsystems/SubEndEffector.h"
#include "commands/CoralCommands.h"
>>>>>>> Intake

RobotContainer::RobotContainer() {
  SubDrivebase::GetInstance().SetDefaultCommand(SubDrivebase::GetInstance().JoystickDrive(_driverController));
  SubVision::GetInstance();

  // Default Commands
  SubDrivebase::GetInstance().SetDefaultCommand(SubDrivebase::GetInstance().JoystickDrive(_driverController));
  
  // Trigger Bindings
  ConfigureBindings();

}

void RobotContainer::ConfigureBindings() {
<<<<<<< HEAD
  _driverController.A().WhileTrue(ControllerRumbleRight(_driverController));
  _driverController.A().WhileTrue(ControllerRumbleLeft(_driverController));
=======
  SubIntake& intake = SubIntake::GetInstance();

  _controller.A().WhileTrue(cmd::IntakeFullSequence());
  _controller.A().OnFalse(SubEndEffector::GetInstance().StopMotor().AlongWith(intake.Stow()));
  _controller.B().WhileTrue(intake.Outtake());
  _controller.X().WhileTrue(intake.Deploy());
  _controller.Y().WhileTrue(intake.Stow());
>>>>>>> Intake
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}

// controller rumble function
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


