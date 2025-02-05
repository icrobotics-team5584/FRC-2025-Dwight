// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "subsystems/SubElevator.h"
#include "commands/GamePieceCommands.h"
#include "frc/DataLogManager.h"
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <grpl/LaserCan.h>
#include <frc/TimedRobot.h>

Robot::Robot() {
  frc::DataLogManager::Start();
    lc = new grpl::LaserCan(0);
    // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
  lc->set_ranging_mode(grpl::LaserCanRangingMode::Long);
  lc->set_timing_budget(grpl::LaserCanTimingBudget::TB100ms);
  lc->set_roi(grpl::LaserCanROI{ 8, 8, 16, 16 });
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  cmd::AutoAdjustElevatorHeight(3);


std::optional<grpl::LaserCanMeasurement> measurement = lc->get_measurement();

if (measurement.has_value() && measurement.value().status == grpl::LASERCAN_STATUS_VALID_MEASUREMENT) {
    // Assuming measurement.value() has a `distance` or similar numeric field
    double offset = measurement.value().distance * 0.22; // Multiply by 0.22 and store as offset
    std::cout << "Offset calculated: " << offset << std::endl;
}
// If no valid measurement, do nothing
  
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