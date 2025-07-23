// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc2/command/Commands.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubClimber.h"
#include "subsystems/SubVision.h"
#include "commands/VisionCommand.h"
#include "commands/DriveCommands.h"
#include "commands/AutonCommands.h"
#include "commands/GamePieceCommands.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Filesystem.h>
#include <wpinet/WebServer.h>
#include "subsystems/SubElevator.h"
#include "subsystems/SubEndEffector.h"
#include "subsystems/SubFunnel.h"
#include "utilities/LEDHelper.h"

RobotContainer::RobotContainer() {
  wpi::WebServer::GetInstance().Start(5800, frc::filesystem::GetDeployDirectory());
  SubVision::GetInstance();
  SubEndEffector::GetInstance();

  // registar named commands
  pathplanner::NamedCommands::registerCommand("ScoreLeft-WithVision", cmd::ScoreWithVision(SubVision::Side::Left));
  pathplanner::NamedCommands::registerCommand("ScoreRight-WithVision", cmd::ScoreWithVision(SubVision::Side::Right));
  pathplanner::NamedCommands::registerCommand("ScoreLeft-WithPrescription", cmd::ScoreWithPrescription(SubVision::Side::Left));
  pathplanner::NamedCommands::registerCommand("ScoringElevatorCleanUp", SubElevator::GetInstance().CmdSetSource());
  pathplanner::NamedCommands::registerCommand("ScoreLeft", cmd::Score(1));
  pathplanner::NamedCommands::registerCommand("ScoreRight", cmd::Score(2));
  pathplanner::NamedCommands::registerCommand("SetElevatorL4", SubElevator::GetInstance().CmdSetL4());
  pathplanner::NamedCommands::registerCommand("BeginSourceIntake", cmd::AutonBeginSourceIntake());
  pathplanner::NamedCommands::registerCommand("EndSourceIntake", cmd::AutonEndSourceIntake());

  pathplanner::NamedCommands::registerCommand("IntakeSource", cmd::IntakeFromSource());
  pathplanner::NamedCommands::registerCommand("AutonSubSystemsZeroSequence", cmd::AutonSubSystemsZeroSequence());
  
  // Default Commands
  SubDrivebase::GetInstance().SetDefaultCommand(cmd::TeleopDrive(_driverController));
  SubVision::GetInstance().SetDefaultCommand(cmd::AddVisionMeasurement());
  // SubEndEffector::GetInstance().SetDefaultCommand(SubEndEffector::GetInstance().KeepCoralInEndEffector()); 

  

  // Trigger Bindings
  ConfigureBindings();

  // AutoChooser options
  _autoChooser.SetDefaultOption("Default-Left", "Default-Score3L4-Vision");

  // main autons
  _autoChooser.AddOption("Default-Left", "Default-Score3L4-Vision");
  _autoChooser.AddOption("Default-Right", "Right-Score3L4-Vision");
  _autoChooser.AddOption("DefaultMiddle-ScoreLeft", "Default-Score1L4-G-Vision");
  _autoChooser.AddOption("DefaultMiddle-ScoreRight", "Default-Score1L4-H-Vision");
  _autoChooser.AddOption("Default-Move-Forward-4m-0.1ms", "MoveForward-4M-0.1ms");

  // tuning autons
  // _autoChooser.AddOption("L-Shape", "L-Shape");
  // _autoChooser.AddOption("L-Shape-Slow", "L-Shape-Slow");
  // _autoChooser.AddOption("L-Shape-Spinning", "L-Shape-Spinning");
  // _autoChooser.AddOption("L-Shape-Spinning-Slow", "L-Shape-Spinning-Slow");
  // _autoChooser.AddOption("WheelCharecterisation-4m-0.1ms", "MoveForward-4M-0.1ms");
  // _autoChooser.AddOption("WheelCharecterisation-4m-0.5ms", "MoveForward-4M-0.5ms");
  // _autoChooser.AddOption("WheelCharecterisation-4m-1.0ms", "MoveForward-4M-1.0ms");
  // _autoChooser.AddOption("WheelCharecterisation-4m-1.5ms", "MoveForward-4M-1.5ms");
  // _autoChooser.AddOption("WheelCharecterisation-4m-2.0ms", "MoveForward-4M-2.0ms");
  // _autoChooser.AddOption("WheelCharecterisation-4m-0.1ms-360rot", "MoveForwardRotate-4M-0.1ms");
  // _autoChooser.AddOption("WheelCharecterisation-4m-1.0ms-360rot", "MoveForwardRotate-4M-1.0ms");
  // _autoChooser.AddOption("WheelCharecterisation-7ishm-0.1ms", "MoveForward-7ishM-0.1ms");
  // _autoChooser.AddOption("SpinInSpot-180", "SpinInSpot-180");
  // _autoChooser.AddOption("SpinInSpot-180-Slow", "SpinInSpot-180-Slow");
  // _autoChooser.AddOption("SpinInSpot-360", "SpinInSpot-360");
  // _autoChooser.AddOption("SpinInSpot-360-Slow", "SpinInSpot360-Slow");

  frc::SmartDashboard::PutData("Chosen Auton", &_autoChooser);

  // Load all auton paths
  defaultLeft = std::make_shared<frc2::CommandPtr>(pathplanner::PathPlannerAuto("Default-Score3L4-Vision").ToPtr());
  defaultRight = std::make_shared<frc2::CommandPtr>(pathplanner::PathPlannerAuto("Right-Score3L4-Vision").ToPtr());
  defaultMiddleScoreLeft = std::make_shared<frc2::CommandPtr>(pathplanner::PathPlannerAuto("Default-Score1L4-G-Vision").ToPtr());
  defaultMiddleScoreRight = std::make_shared<frc2::CommandPtr>(pathplanner::PathPlannerAuto("Default-Score1L4-H-Vision").ToPtr());
  moveForward = std::make_shared<frc2::CommandPtr>(pathplanner::PathPlannerAuto("MoveForward-4M-0.1ms").ToPtr());
}

std::shared_ptr<frc2::CommandPtr> RobotContainer::GetAutonomousCommand() {
  // return pathplanner::PathPlannerAuto("test auto").ToPtr();
  auto chosen = _autoChooser.GetSelected();
  if (chosen == "Default-Score3L4-Vision") {
    return defaultLeft;
  }
  if (chosen == "Right-Score3L4-Vision") {
    return defaultRight;
  }
  if (chosen == "Default-Score1L4-G-Vision") {
    return defaultMiddleScoreLeft;
  }
  if (chosen == "Default-Score1L4-H-Vision") {
    return defaultMiddleScoreRight;
  }
  if (chosen == "MoveForward-4M-0.1ms") {
    return moveForward;
  }
  return defaultLeft;
}

void RobotContainer::ConfigureBindings() {
  (_driverController.RightBumper() && _driverController.Y()).WhileTrue(cmd::TeleAlignAndShootAndElevator(SubVision::Side::Right, 4));
  (_driverController.RightBumper() && _driverController.X()).WhileTrue(cmd::TeleAlignAndShootAndElevator(SubVision::Side::Right, 3));
  (_driverController.RightBumper() && _driverController.A()).WhileTrue(cmd::TeleAlignAndShootAndElevator(SubVision::Side::Right, 2));

  (_driverController.LeftBumper() && _driverController.Y()).WhileTrue(cmd::TeleAlignAndShootAndElevator(SubVision::Side::Left, 4));
  (_driverController.LeftBumper() && _driverController.X()).WhileTrue(cmd::TeleAlignAndShootAndElevator(SubVision::Side::Left, 3));
  (_driverController.LeftBumper() && _driverController.A()).WhileTrue(cmd::TeleAlignAndShootAndElevator(SubVision::Side::Left, 2));

  (_driverController.LeftBumper() && _driverController.RightBumper()).WhileTrue(SubDrivebase::GetInstance().GyroCoralStationAlign(_driverController));
  _driverController.Start().OnTrue(SubElevator::GetInstance().ElevatorAutoReset());

  _driverController.B().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());

  _driverController.LeftTrigger().WhileTrue(cmd::IntakeFromSource());
  _driverController.RightTrigger().WhileTrue(SubEndEffector::GetInstance().Shoot());

  // Triggers
  SubDrivebase::GetInstance().CheckCoastButton().ToggleOnTrue(cmd::ToggleBrakeCoast());
  (SubDrivebase::GetInstance().IsTipping() && !SubClimber::GetInstance().IsClimbing())
      .OnTrue(SubElevator::GetInstance().CmdSetSource());
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