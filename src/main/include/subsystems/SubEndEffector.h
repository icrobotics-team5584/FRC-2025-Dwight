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

class SubEndEffector : public frc2::SubsystemBase {
 public:
  SubEndEffector();
  static SubEndEffector &GetInstance() {
    static SubEndEffector inst;
    return inst;
  }

  frc2::CommandPtr EndEffectorIntake();
  frc2::CommandPtr EndEffectorOuttake();
  frc2::CommandPtr EndEffectorStartMotor();
  frc2::CommandPtr EndEffectorReverseMotor();
  frc2::CommandPtr EndEffectorStopMotor();
  bool CheckLineBreak();


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  frc::DigitalInput EndEffectorLineBreak{dio::EndEffectorLineBreak};
  ICSparkMax EndEffectorMotor{canid::EndEffectorMotor, 30_A};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
