// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/SubElevator.h"
#include "commands/GamePieceCommands.h"
#include <frc2/command/Commands.h>
#include "subsystems/SubDrivebase.h"

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubClimber.h"

#include "subsystems/SubVision.h"
#include "commands/VisionCommand.h"
#include "commands/DriveCommands.h"
#include "commands/AutonCommands.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/SubEndEffector.h"
#include "commands/VisionCommand.h"
#include <frc/Filesystem.h>
#include <wpinet/WebServer.h>
#include "subsystems/SubFunnel.h"

RobotContainer::RobotContainer() {
  wpi::WebServer::GetInstance().Start(5800, frc::filesystem::GetDeployDirectory());
  SubVision::GetInstance();
  // SubIntake::GetInstance(); -- dont get instance since the electronics dont exist on the bot yet, and we dont want warnings about it
  SubEndEffector::GetInstance();

  // registar named commands
  pathplanner::NamedCommands::registerCommand("ScoreLeft-WithVision", cmd::ScoreWithVision(1));
  pathplanner::NamedCommands::registerCommand("ScoreRight-WithVision", cmd::ScoreWithVision(2));
  pathplanner::NamedCommands::registerCommand("ScoreLeft", cmd::Score(1));
  pathplanner::NamedCommands::registerCommand("ScoreRight", cmd::Score(2));
  pathplanner::NamedCommands::registerCommand("SetElevatorL4", SubElevator::GetInstance().CmdSetL4());
  
  //.AndThen(cmd::ForceAlignWithTarget(1, _driverController).WithName("AutonAlignToSource"))
  pathplanner::NamedCommands::registerCommand("IntakeSource-WithVision", cmd::IntakeSourceWithVision());
  pathplanner::NamedCommands::registerCommand("IntakeSource", cmd::IntakeFromSource());
  pathplanner::NamedCommands::registerCommand("AutonSubSystemsZeroSequence", cmd::AutonSubSystemsZeroSequence());
  
  // Default Commands
  SubDrivebase::GetInstance().SetDefaultCommand(
      SubDrivebase::GetInstance().JoystickDrive(_driverController));
  SubVision::GetInstance().SetDefaultCommand(cmd::AddVisionMeasurement());
  
  // Trigger Bindings
  ConfigureBindings();

  //Initialise camera object(s)
  _cameraStream = frc::CameraServer::StartAutomaticCapture("Camera Stream", 0); 

  // sidechooser options 
  _sideChooser.SetDefaultOption("Left Side", false);
  _sideChooser.AddOption("Left Side", false);
  _sideChooser.AddOption("Right Side", true);

  // AutoChooser options
  _autoChooser.SetDefaultOption("Default-Move-Forward-4m-0.1ms", "MoveForward-4M-0.1ms"); // default
  _autoChooser.AddOption("Default-Left", "Default-Score3L4-Vision"); // auton testing
  _autoChooser.AddOption("L-Shape", "L-Shape");
  _autoChooser.AddOption("L-Shape-Slow", "L-Shape-Slow");
  _autoChooser.AddOption("L-Shape-Spinning", "L-Shape-Spinning");
  _autoChooser.AddOption("L-Shape-Spinning-Slow", "L-Shape-Spinning-Slow");

  _autoChooser.AddOption("WheelCharecterisation-4m-0.1ms", "MoveForward-4M-0.1ms");
  _autoChooser.AddOption("WheelCharecterisation-4m-0.5ms", "MoveForward-4M-0.5ms");
  _autoChooser.AddOption("WheelCharecterisation-4m-1.0ms", "MoveForward-4M-1.0ms");
  _autoChooser.AddOption("WheelCharecterisation-4m-1.5ms", "MoveForward-4M-1.5ms");
  _autoChooser.AddOption("WheelCharecterisation-4m-2.0ms", "MoveForward-4M-2.0ms");
  _autoChooser.AddOption("WheelCharecterisation-4m-0.1ms-360rot", "MoveForwardRotate-4M-0.1ms");
  _autoChooser.AddOption("WheelCharecterisation-4m-1.0ms-360rot", "MoveForwardRotate-4M-1.0ms");
  _autoChooser.AddOption("WheelCharecterisation-7ishm-0.1ms", "MoveForward-7ishM-0.1ms");

  _autoChooser.AddOption("SpinInSpot-180", "SpinInSpot-180");
  _autoChooser.AddOption("SpinInSpot-180-Slow", "SpinInSpot-180-Slow");
  _autoChooser.AddOption("SpinInSpot-360", "SpinInSpot-360");
  _autoChooser.AddOption("SpinInSpot-360-Slow", "SpinInSpot360-Slow");

  frc::SmartDashboard::PutData("Chosen Auton", &_autoChooser);
  frc::SmartDashboard::PutData("Chosen Side", &_sideChooser);
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  // return pathplanner::PathPlannerAuto("test auto").ToPtr();
  auto _autoSelected = _autoChooser.GetSelected();
  bool _sideSelected = _sideChooser.GetSelected();
  return pathplanner::PathPlannerAuto(_autoSelected, _sideSelected).ToPtr();
}

void RobotContainer::ConfigureBindings() {
  // _driverController.A().ToggleOnTrue(frc2::cmd::StartEnd(
  //   [this] { _cameraStream.SetPath("/dev/video1"); }, //Toggle to second camera (climb cam)
  //   [this] { _cameraStream.SetPath("/dev/video0"); } //Toggle to first camera (drive cam)
  // ));
  _driverController.X().OnTrue(SubDrivebase::GetInstance().SyncSensorBut());
  _driverController.Y().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());
  _driverController.A().WhileTrue(cmd::RemoveAlgaeLow());
  _driverController.B().WhileTrue(cmd::RemoveAlgaeHigh());
  _driverController.RightTrigger().WhileTrue(cmd::ForceAlignWithTarget(2));
  _driverController.LeftTrigger().WhileTrue(cmd::ForceAlignWithTarget(1));
  _driverController.LeftBumper().WhileTrue(cmd::IntakeFromSource());
  _driverController.RightBumper().WhileTrue(SubEndEffector::GetInstance().ScoreCoral());
  
  // Triggers
  SubDrivebase::GetInstance().CheckCoastButton().ToggleOnTrue(cmd::ToggleBrakeCoast());
  SubDrivebase::GetInstance().IsTipping().OnTrue(SubElevator::GetInstance().CmdSetSource());

  //Opperator
  _operatorController.AxisGreaterThan(frc::XboxController::Axis::kLeftY, 0.2)
    .WhileTrue(SubElevator::GetInstance().ManualElevatorMovementDOWN());

  _operatorController.AxisLessThan(frc::XboxController::Axis::kLeftY, -0.2)
    .WhileTrue(SubElevator::GetInstance().ManualElevatorMovementUP());

  frc2::Trigger(frc2::CommandScheduler::GetInstance().GetDefaultButtonLoop(), [=, this] {
    return (_operatorController.GetLeftY() < -0.2);
  }).WhileTrue(SubElevator::GetInstance().ManualElevatorMovementUP());
 
  _operatorController.A().OnTrue(SubElevator::GetInstance().CmdSetL1());
  _operatorController.X().OnTrue(SubElevator::GetInstance().CmdSetL2());
  _operatorController.B().OnTrue(SubElevator::GetInstance().CmdSetL3());
  _operatorController.Y().OnTrue(SubElevator::GetInstance().CmdSetL4());

  _operatorController.POVLeft().OnTrue(SubElevator::GetInstance().ElevatorAutoReset());
  _operatorController.POVRight().OnTrue(SubElevator::GetInstance().CmdSetSource());
  // _operatorController.POVDown.OnTrue()

  _operatorController.LeftTrigger().WhileTrue(cmd::IntakeFromSource());
  _operatorController.RightTrigger().WhileTrue(SubEndEffector::GetInstance().ScoreCoral());
  _operatorController.RightBumper().WhileTrue(cmd::Outtake());

  //  _cameraStream = frc::CameraServer::StartAutomaticCapture("Camera Stream", 0); //Initialise
  //  camera object

  // Rumble controller when end effector line break triggers
  //  SubEndEffector::GetInstance().CheckLineBreakTriggerHigher().OnFalse(ControllerRumbleRight(_driverController).WithTimeout(0.1_s));
  SubEndEffector::GetInstance().CheckLineBreakTriggerLower().OnFalse(
      ControllerRumbleLeft(_driverController).WithTimeout(0.1_s));
}

// Controller rumble functions
frc2::CommandPtr RobotContainer::ControllerRumbleLeft(frc2::CommandXboxController& controller) {
  return frc2::cmd::StartEnd(
      [this, &controller] {
        controller.SetRumble(frc::XboxController::RumbleType::kRightRumble, 1.0);
      },
      [this, &controller] {
        controller.SetRumble(frc::XboxController::RumbleType::kRightRumble, 0);
      });
}

frc2::CommandPtr RobotContainer::ControllerRumbleRight(frc2::CommandXboxController& controller) {
  return frc2::cmd::StartEnd(
      [this, &controller] {
        controller.SetRumble(frc::XboxController::RumbleType::kLeftRumble, 1.0);
      },
      [this, &controller] {
        controller.SetRumble(frc::XboxController::RumbleType::kLeftRumble, 0);
      });
}