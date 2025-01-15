// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/SubElevator.h"
#include "commands/CoralCommands.h"
#include <frc2/command/Commands.h>
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/SubEndEffector.h"
#include "subsystems/SubIntake.h"

RobotContainer::RobotContainer() {
  SubVision::GetInstance();
  SubIntake::GetInstance();

  // Default Commands
  SubDrivebase::GetInstance().SetDefaultCommand(SubDrivebase::GetInstance().JoystickDrive(_driverController));
  
  // Trigger Bindings
  ConfigureBindings();
};


void RobotContainer::ConfigureBindings() {
  //Driver

  //Triggers
  _operatorController.LeftTrigger().WhileTrue(SubEndEffector::GetInstance().IntakeFromSource());
  _operatorController.LeftTrigger().OnFalse(SubEndEffector::GetInstance().StopMotor());
  _operatorController.RightTrigger().WhileTrue(SubEndEffector::GetInstance().IntakeFromGround());
  _operatorController.RightTrigger().OnFalse(SubEndEffector::GetInstance().StopMotor());
  _operatorController.POVRight().OnTrue(SubEndEffector::GetInstance().ScoreCoral());

  //Bumpers


  //Letters
  _driverController.A().WhileTrue(SubDrivebase::GetInstance().WheelCharecterisationCmd()); //Wheel characterisation
  _driverController.Y().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());
  _driverController.X().OnTrue(SubDrivebase::GetInstance().SyncSensorBut());
  _driverController.B().ToggleOnTrue(frc2::cmd::StartEnd(
    [this] { _cameraStream.SetPath("/dev/video1"); }, //Toggle to second camera (climb cam)
    [this] { _cameraStream.SetPath("/dev/video0"); } //Toggle to first camera (drive cam)
  ));

//POV
  //Opperator

  //Triggers

  //Joystick
  frc2::Trigger(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop(), [=, this] {
    return (_operatorController.GetLeftY() < -0.2);
  }).WhileTrue(SubElevator::GetInstance().ManualElevatorMovementDOWN());

  frc2::Trigger(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop(), [=, this] {
    return (_operatorController.GetLeftY() > 0.2);
  }).WhileTrue(SubElevator::GetInstance().ManualElevatorMovementUP());


  //Bumpers

  //Letters
   _operatorController.A().OnTrue(SubElevator::GetInstance().CmdSetL1());
   _operatorController.X().OnTrue(SubElevator::GetInstance().CmdSetL2());
   _operatorController.B().OnTrue(SubElevator::GetInstance().CmdSetL3());
   _operatorController.Y().OnTrue(SubElevator::GetInstance().CmdSetL4());
   _operatorController.LeftBumper().OnTrue(SubElevator::GetInstance().ElevatorAutoReset());
  //  _operatorController.RightBumper().OnTrue(SubElevator::GetInstance().ZeroElevator().IgnoringDisable(true));
   _operatorController.RightBumper().WhileTrue(SubElevator::GetInstance().Climb());
  //POV
  _operatorController.POVUp().WhileTrue(SubElevator::GetInstance().ManualElevatorMovementUP());
  _operatorController.POVDown().WhileTrue(SubElevator::GetInstance().ManualElevatorMovementDOWN());
  _operatorController.POVLeft().WhileTrue(SubElevator::GetInstance().CmdSetSource());

  



  _cameraStream = frc::CameraServer::StartAutomaticCapture("Camera Stream", 0); //Initialise camera object

  //Rumble controller when end effector line break triggers
  SubEndEffector::GetInstance().CheckLineBreakTriggerHigher().OnFalse(ControllerRumbleRight(_driverController).WithTimeout(0.1_s));
  SubEndEffector::GetInstance().CheckLineBreakTriggerLower().OnFalse(ControllerRumbleLeft(_driverController).WithTimeout(0.1_s));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print(".");
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