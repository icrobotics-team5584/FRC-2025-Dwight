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
#include <frc2/command/button/Trigger.h>

class SubEndEffector : public frc2::SubsystemBase {
 public:
  SubEndEffector();
  static SubEndEffector &GetInstance() {
    static SubEndEffector inst;
    return inst;
  }

  frc2::CommandPtr IntakeFromSource();
  frc2::CommandPtr IntakeFromGround();
  frc2::CommandPtr FeedUp();
  frc2::CommandPtr FeedDown();
  frc2::CommandPtr StopMotor();
  bool CheckLineBreak();
  bool LineBreakDownSignal();
  frc2::Trigger CheckLineBreakTrigger();


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  frc::DigitalInput _endEffectorLineBreak{dio::EndEffectorLineBreak};
  ICSparkMax _endEffectorMotor{canid::EndEffectorMotor, 30_A};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
