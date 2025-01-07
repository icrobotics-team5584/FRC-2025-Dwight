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
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

class SubIntake : public frc2::SubsystemBase {
 public:
  SubIntake();
  static SubIntake &GetInstance() {
    static SubIntake inst;
    return inst;
  }

  frc2::CommandPtr Intake();
  frc2::CommandPtr Outtake();
  frc2::CommandPtr Deploy();
  frc2::CommandPtr Stow();
  void SetDesiredAngle(units::degree_t angle);

  const double P = 0;
  const double I = 0;
  const double D = 0;
  const double GearRatio = 1;
  
  


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
 ICSparkMax _intakeLeftMotor{canid::IntakeLeftMotor, 30_A};
 ICSparkMax _intakeRightMotor{canid::IntakeRightMotor, 30_A};

 ctre::phoenix6::configs::TalonFXConfiguration _configIntakePivotMotor{};
 ctre::phoenix6::hardware::TalonFX _intakePivotMotor{canid::IntakePivotMotor};


 
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
