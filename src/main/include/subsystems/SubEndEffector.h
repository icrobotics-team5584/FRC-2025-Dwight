// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/ICSparkFlex.h"
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
  
  frc2::CommandPtr FeedUp();
  frc2::CommandPtr FeedUpSLOW();
  frc2::CommandPtr FeedDown();
  frc2::CommandPtr FeedDownSLOW();
  frc2::CommandPtr StopMotor();
  frc2::CommandPtr ScoreCoral();
  frc2::CommandPtr Shoot();
  frc2::CommandPtr ScoreCoralSLOW();
  frc2::CommandPtr KeepCoralInEndEffector();
  frc2::CommandPtr RemoveAlgae();
  
  bool CheckLineBreakHigher();
  bool CheckLineBreakLower();
  bool IsCoralSecure();
  bool LineBreakDownSignal();
  frc2::Trigger CheckLineBreakTriggerHigher();
  frc2::Trigger CheckLineBreakTriggerLower();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

 private:
  frc::DigitalInput _endEffectorLineBreakHigher{dio::ENDEFFECTOR_LINEBREAK_HIGHER};
  frc::DigitalInput _endEffectorLineBreakLower{dio::ENDEFFECTOR_LINEBREAK_LOWER};
  ICSparkFlex _endEffectorMotor{canid::ENDEFFECTOR_MOTOR, 30_A};
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
