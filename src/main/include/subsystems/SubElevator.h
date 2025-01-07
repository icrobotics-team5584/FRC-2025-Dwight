// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "utilities/ICSparkMax.h"
#include "utilities/ICSparkEncoder.h"
#include <frc/simulation/ElevatorSim.h>

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

  units::turn_t RotationsFromHeight(units::meter_t height);

  void Periodic() override;
  void SimulationPeriodic() override;


 private:
  ctre::phoenix6::hardware::TalonFX _ElevatorMotor1 {13};
  ctre::phoenix6::hardware::TalonFX _ElevatorMotor2 {14};

  static constexpr double _P = 0;
  static constexpr double _I = 0;
  static constexpr double _D = 0;
  static constexpr double _V = 0;
  static constexpr double _GEAR_RATIO = 20;
  static constexpr units::meter_t _DRUM_RADIUS = 4_cm;
  static constexpr units::meter_t _MAX_HEIGHT = 2_m;
  static constexpr units::meter_t _MIN_HEIGHT = 0.5_m;
  static constexpr units::meter_t _START_HEIGHT = 0.4_m;
  static constexpr units::kilogram_t _CARRIAGE_MASS = 6_kg;





  //   //Simulation stuff
  frc::sim::ElevatorSim _motorSim{frc::DCMotor::Falcon500(2), _GEAR_RATIO, _CARRIAGE_MASS, _DRUM_RADIUS, _MIN_HEIGHT, _MAX_HEIGHT, true, _START_HEIGHT};
};
