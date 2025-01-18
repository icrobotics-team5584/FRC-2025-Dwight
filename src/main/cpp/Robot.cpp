// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "subsystems/SubElevator.h"
#include "frc/DataLogManager.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include "grpl/CanBridge.h"
#include <iostream>

Robot::Robot() {
  grpl::start_can_bridge();
  lc = new grpl::LaserCan(1);
    // Initialise the settings of the LaserCAN, 16x16 range
  //lc->set_ranging_mode(grpl::LaserCanRangingMode::Long);
  //lc->set_timing_budget(grpl::LaserCanTimingBudget::TimingBudget100ms);
  //lc->set_roi(grpl::LaserCanROI{ 8, 8, 16, 16 });
  frc::DataLogManager::Start();
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

  std::optional<grpl::LaserCanMeasurement> measurement = lc->get_measurement();
  if (measurement.has_value() && measurement.value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT) {
    frc::SmartDashboard::PutNumber("LaserCan/distance/mm", measurement.value().distance_mm);
  }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }

//Auto climber reset by bringing elevator to zero position then reset
  SubElevator::GetInstance().ElevatorAutoReset();
} 

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
