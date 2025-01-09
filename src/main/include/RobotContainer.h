// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc/smartdashboard/SendableChooser.h>

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

 private:
  void ConfigureBindings();
  
  //controllers
  frc2::CommandXboxController _driverController{5};
  frc2::CommandXboxController _operatorController{1};
  frc2::CommandXboxController _tuningController{0};

  frc::SendableChooser<std::string> _autoChooser;
};
