// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubVision.h"
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
  SubVision::GetInstance();

  // Default Commands
  SubDrivebase::GetInstance().SetDefaultCommand(SubDrivebase::GetInstance().JoystickDrive(_driverController));
  
  // Trigger Bindings
  ConfigureBindings();

  _cameraStream = frc::CameraServer::StartAutomaticCapture("Camera Stream", 0); //Initialise camera object
}

void RobotContainer::ConfigureBindings() {
  _driverController.A().WhileTrue(SubDrivebase::GetInstance().WheelCharecterisationCmd()); //Wheel characterisation
  
  _driverController.B().ToggleOnTrue(frc2::cmd::StartEnd(
    [this] { _cameraStream.SetPath("/dev/video1"); }, //Toggle to second camera (climb cam)
    [this] { _cameraStream.SetPath("/dev/video0"); } //Toggle to first camera (drive cam)
  ));
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
