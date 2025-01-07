// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/ICSparkMax.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include "Constants.h"
#include <frc/DigitalInput.h>

class SubIntake : public frc2::SubsystemBase {
 public:
  SubIntake();
  static SubIntake &GetInstance() {
    static SubIntake inst;
    return inst;
  }

  frc2::CommandPtr Intake();
  frc2::CommandPtr Outtake();

  
  


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
 ICSparkMax _intakeLeftMotor{canid::IntakeLeftMotor, 30_A};
 ICSparkMax _intakeRightMotor{canid::IntakeRightMotor, 30_A};

 
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
