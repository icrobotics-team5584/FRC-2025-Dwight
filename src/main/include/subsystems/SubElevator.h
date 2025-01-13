// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "utilities/ICSparkMax.h"
#include "utilities/ICSparkEncoder.h"
#include <frc/simulation/ElevatorSim.h>
#include <frc2/command/button/CommandXboxController.h>
#include <units/constants.h>
#include "Constants.h"

class SubElevator : public frc2::SubsystemBase {
 public:

  static SubElevator& GetInstance() {
  static SubElevator inst;
    return inst;
  }

  SubElevator();

  frc2::CommandPtr CmdSetL1();
  frc2::CommandPtr CmdSetL2();
  frc2::CommandPtr CmdSetL3();
  frc2::CommandPtr CmdSetL4();
  frc2::CommandPtr CmdSetSource();
  frc2::CommandPtr ZeroElevator();
  frc2::CommandPtr ElevatorResetCheck();
  frc2::CommandPtr ElevatorAutoReset();
  frc2::CommandPtr ElevatorStop();
  frc2::CommandPtr ManualElevatorMovementUP();
  frc2::CommandPtr ManualElevatorMovementDOWN();
  frc2::CommandPtr ManualElevatorMovementDOWNSLOW();

  frc2::CommandPtr CmdElevatorToPosition(units::meter_t height);
  // frc2::CommandPtr ElevatorJoystickDrive(frc2::CommandXboxController& _controller);
  units::turn_t RotationsFromHeight(units::meter_t height);
  units::meter_t HeightFromRotations(units::turn_t turns);
  units::turns_per_second_t RotationsFromMetersPerSecond(units::meters_per_second_t meterspersec);
  units::ampere_t GetM1Current();

  void Stop();
  void EnableSoftLimit(bool enabled);


  void Periodic() override;
  void SimulationPeriodic() override;



  //reset
  bool Reseting = false;
  bool Reseted = false;

  bool ResetM1 = false; 



 private:
  ctre::phoenix6::hardware::TalonFX _elevatorMotor1 {canid::elevatorMotor1};
  ctre::phoenix6::hardware::TalonFX _elevatorMotor2 {canid::elevatorMotor2};

  static constexpr double _P = 15; //134.04;
  static constexpr double _I = 0;
  static constexpr double _D = 0;
  static constexpr double _V = 0;                     
  static constexpr double _A = 0;
  static constexpr double _G = 0.15; //8.6704096794128409086;
  static constexpr double _GEAR_RATIO = 14;
  static constexpr units::meter_t _DRUM_RADIUS = 1.84_cm * 2; // effective radius - doubled as 2 stage elevator
  static constexpr units::meter_t _DRUM_CIRCUMFERENCE = _DRUM_RADIUS * 2 * units::constants::pi;
  static constexpr units::meter_t _MAX_HEIGHT = 2_m;
  static constexpr units::meter_t _MIN_HEIGHT = 0.239_m;
  static constexpr units::meter_t _START_HEIGHT = 0_m;
  static constexpr units::kilogram_t _CARRIAGE_MASS = 6_kg;
  static constexpr units::meter_t _L1_HEIGHT = 0.51_m;
  static constexpr units::meter_t _L2_HEIGHT = 0.86_m;
  static constexpr units::meter_t _L3_HEIGHT = 1.26_m;
  static constexpr units::meter_t _L4_HEIGHT = 1.7_m;//1.88
  static constexpr units::meter_t _SOURCE_HEIGHT = 0.93_m;
  static constexpr units::meters_per_second_t _CRUISE_VELOCITY = 1_mps; //0.82; //Adjust
  static constexpr units::meters_per_second_squared_t  _ACCELERATION = 3_mps_sq; //Adjust
  static constexpr double zeroingCurrentLimit = 15;




  //   //Simulation stuff
  frc::sim::ElevatorSim _motorSim{frc::DCMotor::Falcon500(2), _GEAR_RATIO, _CARRIAGE_MASS, _DRUM_RADIUS, _MIN_HEIGHT, _MAX_HEIGHT, true, _START_HEIGHT};
};
