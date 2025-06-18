// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <cameraserver/CameraServer.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>

class RobotContainer {
 public:
  RobotContainer();

  std::shared_ptr<frc2::CommandPtr> GetAutonomousCommand();
  frc2::CommandPtr ControllerRumbleLeft(frc2::CommandXboxController& controller);
  frc2::CommandPtr ControllerRumbleRight(frc2::CommandXboxController& controller);
  frc2::CommandPtr ControllerRumble(frc2::CommandXboxController& controller);


 private:
  void ConfigureBindings();
  
  //controllers
  frc2::CommandXboxController _driverController{0};
  frc2::CommandXboxController _operatorController{1};
  frc2::CommandXboxController _tuningController{5};

  frc::SendableChooser<std::string> _autoChooser;
  frc::SendableChooser<bool> _sideChooser;
  
  cs::UsbCamera _cameraStream; //Camera object

  //Auton paths
  std::shared_ptr<frc2::CommandPtr> defaultLeft;
  std::shared_ptr<frc2::CommandPtr> defaultRight;
  std::shared_ptr<frc2::CommandPtr> defaultMiddleScoreLeft;
  std::shared_ptr<frc2::CommandPtr> defaultMiddleScoreRight;
  std::shared_ptr<frc2::CommandPtr> moveForward;
  std::shared_ptr<frc2::CommandPtr> middlePushScoreRightL4;

  //std::map
};