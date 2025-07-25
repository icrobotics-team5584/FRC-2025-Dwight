// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "utilities/ICSparkFlex.h"
#include "Constants.h"
#include <frc2/command/CommandPtr.h>


class SubFunnel : public frc2::SubsystemBase {
 public:
  SubFunnel();
  static SubFunnel &GetInstance() {
    static SubFunnel inst;
    return inst;
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  frc2::CommandPtr FeedDownFunnel();
  frc2::CommandPtr FeedDownFunnelSLOW();
  frc2::CommandPtr StopFunnelMotor();
  frc2::CommandPtr FeedUpFunnel();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ICSparkFlex _funnelMotor{canid::FUNNEL_MOTOR, 30_A};
};
