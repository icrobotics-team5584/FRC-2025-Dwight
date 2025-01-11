// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "utilities/ICSparkMax.h"
#include "utilities/ICSparkEncoder.h"
#include <frc/simulation/ElevatorSim.h>
#include <units/constants.h>

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

  frc2::CommandPtr CmdElevatorToPosition(units::meter_t height);
  units::turn_t RotationsFromHeight(units::meter_t height);
  units::turns_per_second_t RotationsFromMetersPerSecond(units::meters_per_second_t meterspersec);

  void Periodic() override;
  void SimulationPeriodic() override;


 private:
  ctre::phoenix6::hardware::TalonFX _ElevatorMotor1 {13};
  ctre::phoenix6::hardware::TalonFX _ElevatorMotor2 {14};

  static constexpr double _P = 7; //134.04;
  static constexpr double _I = 0;
  static constexpr double _D = 0;
  static constexpr double _V = 1;
  static constexpr double _A = 0.2;
  static constexpr double _G = 0.22; //8.6704096794128409086;
  static constexpr double _GEAR_RATIO = 14;
  static constexpr units::meter_t _DRUM_RADIUS = 4_cm;
  static constexpr units::meter_t _DRUM_CIRCUMFERENCE = _DRUM_RADIUS * 2 * units::constants::pi;
  static constexpr units::meter_t _MAX_HEIGHT = 2_m;
  static constexpr units::meter_t _MIN_HEIGHT = 0_m;
  static constexpr units::meter_t _START_HEIGHT = 0_m;
  static constexpr units::kilogram_t _CARRIAGE_MASS = 6_kg;
  static constexpr units::meter_t _L1_HEIGHT = 0.51_m;
  static constexpr units::meter_t _L2_HEIGHT = 0.86_m;
  static constexpr units::meter_t _L3_HEIGHT = 1.26_m;
  static constexpr units::meter_t _L4_HEIGHT = 1.88_m;
  static constexpr units::meter_t _SOURCE_HEIGHT = 0.93_m;
  static constexpr units::meters_per_second_t _CRUISE_VELOCITY = 1_mps; //0.82; //Adjust
  static constexpr units::meters_per_second_squared_t  _ACCELERATION = 3_mps_sq; //Adjust
  




  //   //Simulation stuff
  frc::sim::ElevatorSim _motorSim{frc::DCMotor::Falcon500(2), _GEAR_RATIO, _CARRIAGE_MASS, _DRUM_RADIUS, _MIN_HEIGHT, _MAX_HEIGHT, true, _START_HEIGHT};
};
