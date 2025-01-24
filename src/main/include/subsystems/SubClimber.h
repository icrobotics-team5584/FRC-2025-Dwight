// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h> 
#include <frc2/command/Commands.h>


// utilities
#include "utilities/ICSpark.h"
#include "utilities/ICSparkFlex.h"
#include "utilities/ICSparkMax.h"
#include <units/current.h>
#include "Constants.h"
#include <frc/simulation/DCMotorSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <units/angular_velocity.h>
#include "utilities/RobotLogs.h"

class SubClimber : public frc2::SubsystemBase {
 public:
  static SubClimber& GetInstance() {static SubClimber inst; return inst;}
  SubClimber();

  const double P = 10;
  const double I = 0;
  const double D = 0;
  const double GEAR_RATIO = 480; //assuming 480 means reduction and not 1/480

  frc2::CommandPtr ClimberAutoReset();
  frc2::CommandPtr ManualClimberMovementUP();
  frc2::CommandPtr ManualClimberMovementDOWN();
  frc2::CommandPtr ManualClimberMovementDOWNSLOW();
  frc2::CommandPtr ClimberResetCheck();
  units::ampere_t  GetM1Current();
  frc2::CommandPtr StowClimber();
  frc2::CommandPtr ReadyClimber();
  frc2::CommandPtr Climb();
  frc2::CommandPtr set12V();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

  bool Reseting = false;
  bool Reseted = false;
  bool ResetM1 = false; 
  static constexpr units::ampere_t zeroingCurrentLimit = 20_A;
  static constexpr units::turn_t PREPARE_TURNS = -75_deg;//get numbers later 
  static constexpr units::turn_t CLIMB_TURNS = 45_deg; 
  static constexpr units::turn_t STOW_TURNS = 90_deg; 

  //sim
  const units::length::meter_t ARM_LENGTH = 0.59_m; //0.59m
  const units::mass::kilogram_t ARM_MASS = 1_kg; //55kg robot weight
  const units::angle::degree_t ARM_MIN_ANGLE = -360_deg; //-90
  const units::angle::degree_t ARM_MAX_ANGLE = 100_tr; //-30
  const units::angle::degree_t ARM_HOME_ANGLE = 0_deg; //-90
  
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ICSparkFlex _climberMotor{canid::climberMotor, 40_A}; // _A is amps; 20 amps may be too low for the climber to function

  frc::sim::SingleJointedArmSim _climberSim{frc::DCMotor::NEO(1), GEAR_RATIO, frc::sim::SingleJointedArmSim::EstimateMOI(ARM_LENGTH, ARM_MASS), ARM_LENGTH, ARM_MIN_ANGLE, ARM_MAX_ANGLE, false, ARM_HOME_ANGLE};
  
  frc::Mechanism2d _singleJointedArmMech{3, 3};  // canvas width and height
  frc::MechanismRoot2d* _armRoot = _singleJointedArmMech.GetRoot("armRoot", 1, 1);  // root x and y
  frc::MechanismLigament2d* _arm1Ligament =
    _armRoot->Append<frc::MechanismLigament2d>("ligament2", ARM_LENGTH.value(), 0_deg);

};