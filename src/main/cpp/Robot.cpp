// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include "subsystems/SubElevator.h"
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include "utilities/RobotLogs.h"
#include "URCL.h"
#include <frc/RobotController.h>
#include "utilities/LEDHelper.h"

Robot::Robot() {
  frc::DataLogManager::Start();
  frc::SmartDashboard::PutData( &frc2::CommandScheduler::GetInstance() );
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
  URCL::Start(std::map<int, std::string_view>{{canid::CLIMBER_MOTOR, "Climber"},
                                              {canid::ENDEFFECTOR_MOTOR, "EndEffector"},
                                              {canid::FUNNEL_MOTOR, "Funnel"}});
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
  Logger::Log("Robot/RioBrownOut", frc::RobotController::IsBrownedOut());
  Logger::Log("Robot/RioInputVoltage", frc::RobotController::GetInputVoltage());
  Logger::Log("Robot/RioInputCurrent", frc::RobotController::GetInputCurrent());
  Logger::Log("Robot/BatteryVoltage", frc::RobotController::GetBatteryVoltage());
  Logger::Log("Robot/PDHInputVoltage", frc::RobotController::GetInputVoltage());
  Logger::Log("Robot/PDHTotalCurrent", frc::RobotController::GetInputCurrent());
  // frc::CANStatus canStatus = frc::RobotController::GetCANStatus();
  // Logger::Log("Robot/CANBusUtilization", canStatus.percentBusUtilization);
  // Logger::Log("Robot/CANBusOffCount", canStatus.busOffCount);
  // Logger::Log("Robot/CANTxFullCount", canStatus.txFullCount);
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
