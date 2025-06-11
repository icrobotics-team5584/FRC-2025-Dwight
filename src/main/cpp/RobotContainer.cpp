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
  // pathplanner::NamedCommands::registerCommand("ScoreLeft-WithPrescription", cmd::ScoreWithPrescription(SubVision::Side::Left));
  pathplanner::NamedCommands::registerCommand("ScoringElevatorCleanUp", SubElevator::GetInstance().CmdSetSource());
  pathplanner::NamedCommands::registerCommand("ScoreLeft", cmd::Score(1));
  pathplanner::NamedCommands::registerCommand("ScoreRight", cmd::Score(2));
  pathplanner::NamedCommands::registerCommand("SetElevatorL4", SubElevator::GetInstance().CmdSetL4());
  pathplanner::NamedCommands::registerCommand("BeginSourceIntake", cmd::AutonBeginSourceIntake());
  pathplanner::NamedCommands::registerCommand("EndSourceIntake", cmd::AutonEndSourceIntake());
  
  //.AndThen(cmd::ForceAlignWithTarget(1, _driverController).WithName("AutonAlignToSource"))
  pathplanner::NamedCommands::registerCommand("IntakeSource", cmd::IntakeFromSource());
  pathplanner::NamedCommands::registerCommand("AutonSubSystemsZeroSequence", cmd::AutonSubSystemsZeroSequence());
  
  // Default Commands
  SubDrivebase::GetInstance().SetDefaultCommand(cmd::TeleopDrive(_driverController));
  SubVision::GetInstance().SetDefaultCommand(cmd::AddVisionMeasurement());
  

  // Trigger Bindings
  ConfigureBindings();

  // AutoChooser options
  _autoChooser.SetDefaultOption("Default-Move-Forward-4m-0.1ms", "MoveForward-4M-0.1ms"); // safety option

  // main autons
  _autoChooser.AddOption("Default-Left", "Default-Score3L4-Vision");
  _autoChooser.AddOption("Default-Right", "Right-Score3L4-Vision");
  _autoChooser.AddOption("DefaultMiddle-ScoreLeft", "Default-Score1L4-G-Vision");
  _autoChooser.AddOption("DefaultMiddle-ScoreRight", "Default-Score1L4-H-Vision");

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
  return moveForward;
}

void RobotContainer::ConfigureBindings() {
  // Driver

  //  _cameraStream = frc::CameraServer::StartAutomaticCapture("Camera Stream", 0); //Initialise camera object
  // _driverController.A().ToggleOnTrue(frc2::cmd::StartEnd(
  //   [this] { _cameraStream.SetPath("/dev/video1"); }, //Toggle to second camera (climb cam)
  //   [this] { _cameraStream.SetPath("/dev/video0"); } //Toggle to first camera (drive cam)
  // ));
  _driverController.A().OnTrue(SubDrivebase::GetInstance().SyncSensorBut());
  _driverController.Y().OnTrue(SubDrivebase::GetInstance().ResetGyroCmd());
  
  // _driverController.X().WhileTrue(SubDrivebase::GetInstance().GyroCoralLeftStationAlign(_driverController));
  // _driverController.B().WhileTrue(SubDrivebase::GetInstance().GyroCoralRightStationAlign(_driverController)); 
  _driverController.RightBumper().WhileTrue(SubDrivebase::GetInstance().GyroCoralLeftStationAlign(_driverController));
  _driverController.LeftBumper().WhileTrue(SubDrivebase::GetInstance().GyroCoralRightStationAlign(_driverController)); 
  // _driverController.B().WhileTrue(cmd::YAlignWithTarget(SubVision::Side::Left));
  // _driverController.X().WhileTrue(cmd::YAlignWithTarget(SubVision::Side::Right));
  _driverController.B().WhileTrue(cmd::TeleAlignAndShoot(SubVision::Side::Right));
  _driverController.X().WhileTrue(cmd::TeleAlignAndShoot(SubVision::Side::Left));
  _driverController.LeftTrigger().WhileTrue(cmd::IntakeFromSource());
  _driverController.RightTrigger().WhileTrue(SubEndEffector::GetInstance().ScoreCoral());

  // Triggers
  SubDrivebase::GetInstance().CheckCoastButton().ToggleOnTrue(cmd::ToggleBrakeCoast());
  (SubDrivebase::GetInstance().IsTipping() && !SubClimber::GetInstance().IsClimbing())
      .OnTrue(SubElevator::GetInstance().CmdSetSource());

  // Operator
  _operatorController.AxisGreaterThan(frc::XboxController::Axis::kLeftY, 0.2)
      .WhileTrue(SubElevator::GetInstance().ManualElevatorMovementDOWN());

  _operatorController.AxisLessThan(frc::XboxController::Axis::kLeftY, -0.2)
      .WhileTrue(SubElevator::GetInstance().ManualElevatorMovementUP());

  (!_operatorController.Back() && _operatorController.A()).OnTrue(cmd::ClimbHalfwaySequence());  // Climb to halfway
  (_operatorController.Back() && _operatorController.A()).OnTrue(cmd::ClimbHalfwaySequence(true));  // Force climb halfway

  (!_operatorController.Back() && _operatorController.X()).OnTrue(SubElevator::GetInstance().CmdSetAutoL2());  // Set AutoScore L2 normally
  (_operatorController.Back() && _operatorController.X()).OnTrue(cmd::SetElevatorL2(true));  // Force set L2

  (!_operatorController.Back() && _operatorController.B()).OnTrue(SubElevator::GetInstance().CmdSetAutoL3());  // Set AutoScore L3 normally
  (_operatorController.Back() && _operatorController.B()).OnTrue(cmd::SetElevatorL3(true));  // Force set L3

  (!_operatorController.Back() && _operatorController.Y()).OnTrue(SubElevator::GetInstance().CmdSetAutoL4());  // Set AutoScore L4 normally
  (_operatorController.Back() && _operatorController.Y()).OnTrue(cmd::SetElevatorL4(true));  // Force set L4

  _operatorController.POVLeft().OnTrue(SubElevator::GetInstance().ElevatorAutoReset());
  _operatorController.POVRight().OnTrue(SubElevator::GetInstance().CmdSetSource());

  (!_operatorController.Back() && _operatorController.POVUp()).OnTrue(cmd::ClimbUpSequence());
  (_operatorController.Back() && _operatorController.POVUp()).OnTrue(cmd::ClimbUpSequence(true)); // Force elevator to move for climb
  
  (!_operatorController.Back() && _operatorController.POVDown()).OnTrue(cmd::ClimbDownSequence());
  (_operatorController.Back() && _operatorController.POVDown()).OnTrue(cmd::ClimbDownSequence(true));

  _operatorController.LeftTrigger().WhileTrue(cmd::RemoveAlgaeLow());
  _operatorController.RightTrigger().WhileTrue(cmd::RemoveAlgaeHigh());
  _operatorController.RightBumper().WhileTrue(cmd::Outtake());
  
  _operatorController.Start().WhileTrue(SubClimber::GetInstance().ClimberAutoReset());
  _operatorController.RightStick().OnTrue(cmd::StowClimber());


  _operatorController.LeftBumper().WhileTrue(cmd::IntakeFromSource());

  // SubEndEffector::GetInstance().CheckLineBreakTriggerLower().OnTrue(LEDHelper::GetInstance().SetDefaultCommand(LEDHelper::GetInstance().SetScrollingRainbow()));
  // SubEndEffector::GetInstance().CheckLineBreakTriggerLower().OnFalse(Helper::GetInstance().SetScrollingRainbow());

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