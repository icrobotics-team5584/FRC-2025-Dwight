// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/SubElevator.h"
#include "commands/GamePieceCommands.h"
#include <frc2/command/Commands.h>
#include "subsystems/SubDrivebase.h"

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include "subsystems/SubDrivebase.h"

#include "subsystems/SubVision.h"
#include "commands/VisionCommand.h"
#include "commands/DriveCommands.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/SubEndEffector.h"
#include "subsystems/SubIntake.h"
#include "commands/VisionCommand.h"
#include <frc/Filesystem.h>
#include <wpinet/WebServer.h>

RobotContainer::RobotContainer() {
  wpi::WebServer::GetInstance().Start(5800, frc::filesystem::GetDeployDirectory());
  SubVision::GetInstance();
  SubIntake::GetInstance();

  // Default Commands
  SubDrivebase::GetInstance().SetDefaultCommand(SubDrivebase::GetInstance().JoystickDrive(_driverController));
  SubVision::GetInstance().SetDefaultCommand(cmd::AddVisionMeasurement());

  // Trigger Bindings
  ConfigureBindings();

  //Initialise camera object(s)
  _cameraStream = frc::CameraServer::StartAutomaticCapture("Camera Stream", 0); 


  // AutoChooser options
  _autoChooser.AddOption("Default-Left-Slowed", "Default-Left-SlowTest"); // auton testing
  _autoChooser.AddOption("Default-Left", "Default-Left");  
  _autoChooser.AddOption("Default-Middle", "placeholder-DM"); // placeholder
  _autoChooser.AddOption("Default-Right", "Default-Right"); 
  _autoChooser.AddOption("TeammateHelper-Left", "placeholder-THL"); // placeholder
  _autoChooser.AddOption("TeammateHelper-Right", "placeholder-THR"); // placeholder
  _autoChooser.AddOption("L-Shape", "L-Shape");
  _autoChooser.AddOption("L-Shape-Slow", "L-Shape-Slow");
  _autoChooser.AddOption("L-Shape-Spinning", "L-Shape-Spinning");
  _autoChooser.AddOption("L-Shape-Spinning-Slow", "L-Shape-Spinning-Slow");
  _autoChooser.AddOption("move", "move");
  _autoChooser.AddOption("WheelCharecterisation-4m-0.1ms", "MoveForward-4M-0.1ms");
  _autoChooser.AddOption("WheelCharecterisation-4m-0.5ms", "MoveForward-4M-0.5ms");
  _autoChooser.AddOption("WheelCharecterisation-4m-1.0ms", "MoveForward-4M-1.0ms");
  _autoChooser.AddOption("WheelCharecterisation-4m-1.5ms", "MoveForward-4M-1.5ms");
  _autoChooser.AddOption("WheelCharecterisation-4m-2.0ms", "MoveForward-4M-2.0ms");
  _autoChooser.AddOption("WheelCharecterisation-7ishm-0.1ms", "MoveForward-7ishM-0.1ms");
  _autoChooser.AddOption("SpinInSpot-180", "SpinInSpot-180");
  _autoChooser.AddOption("SpinInSpot-180-Slow", "SpinInSpot-180-Slow");
  _autoChooser.AddOption("SpinInSpot-360", "SpinInSpot-360");
  _autoChooser.AddOption("SpinInSpot-360-Slow", "SpinInSpot360-Slow");  

  frc::SmartDashboard::PutData("Chosen Auton", &_autoChooser);   
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  //return pathplanner::PathPlannerAuto("test auto").ToPtr();
  auto _autoSelected = _autoChooser.GetSelected();
  units::second_t delay = 0.00_s;
  
  return frc2::cmd::Wait(delay)
    .AndThen(pathplanner::PathPlannerAuto(_autoSelected).ToPtr());
}

void RobotContainer::ConfigureBindings() {
  _driverController.A().WhileTrue(cmd::YAlignWithTarget(1, _driverController));
  _driverController.B().WhileTrue(cmd::YAlignWithTarget(2, _driverController));
  _driverController.LeftTrigger().WhileTrue(SubDrivebase::GetInstance().RobotCentricDrive(_driverController));

  _operatorController.LeftTrigger().WhileTrue(SubEndEffector::GetInstance().IntakeFromSource());
  _operatorController.LeftTrigger().OnFalse(SubEndEffector::GetInstance().StopMotor());
  _operatorController.RightTrigger().WhileTrue(SubEndEffector::GetInstance().IntakeFromGround());
  _operatorController.RightTrigger().OnFalse(SubEndEffector::GetInstance().StopMotor());
  _operatorController.POVRight().WhileTrue(SubEndEffector::GetInstance().ScoreCoral());
  _operatorController.POVLeft().WhileTrue(cmd::RemoveAlgae());
  
  _driverController.Y().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());
  _driverController.X().OnTrue(SubDrivebase::GetInstance().SyncSensorBut());
  // _driverController.B().ToggleOnTrue(frc2::cmd::StartEnd(
  //   [this] { _cameraStream.SetPath("/dev/video1"); }, //Toggle to second camera (climb cam)
  //   [this] { _cameraStream.SetPath("/dev/video0"); } //Toggle to first camera (drive cam)
  // ));
  

  //Opperator

  //Triggers
  SubDrivebase::GetInstance().CheckButton().OnTrue(cmd::toggleBrakeCoast());
  frc2::Trigger(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop(), [=, this] {
    return (_operatorController.GetLeftY() > 0.2);
  }).WhileTrue(SubElevator::GetInstance().ManualElevatorMovementDOWN());

  frc2::Trigger(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop(), [=, this] {
    return (_operatorController.GetLeftY() < -0.2);
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
  // _operatorController.POVUp().WhileTrue(SubElevator::GetInstance().ManualElevatorMovementUP());
  // _operatorController.POVDown().WhileTrue(SubElevator::GetInstance().ManualElevatorMovementDOWN());
  _operatorController.POVDown().WhileTrue(SubElevator::GetInstance().CmdSetSource());

  



   _cameraStream = frc::CameraServer::StartAutomaticCapture("Camera Stream", 0); //Initialise camera object

  //Rumble controller when end effector line break triggers
  SubEndEffector::GetInstance().CheckLineBreakTriggerHigher().OnFalse(ControllerRumbleRight(_driverController).WithTimeout(0.1_s));
  SubEndEffector::GetInstance().CheckLineBreakTriggerLower().OnFalse(ControllerRumbleLeft(_driverController).WithTimeout(0.1_s));
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