// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>

#include <cameraserver/CameraServer.h>

class RobotContainer {
 public:
  RobotContainer();
  frc2::CommandXboxController _driverController{0};
  frc2::CommandPtr GetAutonomousCommand();

 private:
  frc2::CommandXboxController _controller{0};
  void ConfigureBindings();

  cs::UsbCamera _cameraStream; //Camera object
};
