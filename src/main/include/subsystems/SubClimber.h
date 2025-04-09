// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/RobotBase.h>
#include <frc2/command/Commands.h>
#include "utilities/ICSpark.h"
#include "utilities/ICSparkFlex.h"
#include "utilities/ICSparkMax.h"
#include <units/current.h>
#include "Constants.h"
#include <frc/simulation/DCMotorSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc2/command/button/Trigger.h>
#include <units/angular_velocity.h>
#include "utilities/RobotLogs.h"
#include "utilities/MechanismCircle2d.h"

class SubClimber : public frc2::SubsystemBase {
 public:
  static SubClimber& GetInstance() {static SubClimber inst; return inst;}
  SubClimber();

  const double P = 19;
  const double I = 0;
  const double D = 0;
  const double GEAR_RATIO = 600; // 600:1

  frc2::CommandPtr ClimberAutoReset();
  frc2::CommandPtr ManualClimberMovementUP();
  frc2::CommandPtr ManualClimberMovementDOWN();
  frc2::CommandPtr ManualClimberMovementDOWNSLOW();
  frc2::CommandPtr ClimberResetCheck();
  frc2::CommandPtr StowClimber();
  frc2::CommandPtr ReadyClimber();
  frc2::CommandPtr ClimbToHalfway();
  frc2::CommandPtr Climb();
  frc2::CommandPtr set12V();

  units::ampere_t  GetM1Current();
  frc2::Trigger IsClimbing();
  bool IsAtTarget();
  bool IsOut();

  void SetBrakeMode(bool mode);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

  bool _resetting = false;
  bool _hasReset = false; 
  static constexpr units::ampere_t zeroingCurrentLimit = 30_A;
  static constexpr units::turn_t PREPARE_TURNS = 180_deg;//get numbers later 
  static constexpr units::turn_t HALF_CLIMB_TURNS = 0.37_tr;
  static constexpr units::turn_t CLIMB_TURNS = 0.207_tr; // 0.217_tr for climbing all the way in
  static constexpr units::turn_t STOW_TURNS = 1_deg; 

  //sim
  const units::length::meter_t ARM_LENGTH = 0.59_m; //0.59m
  const units::mass::kilogram_t ARM_MASS = 1_kg; //55kg robot weight
  const units::angle::degree_t ARM_MIN_ANGLE = -360_deg; //-90
  const units::angle::degree_t ARM_MAX_ANGLE = 100_tr; //-30
  const units::angle::degree_t ARM_HOME_ANGLE = 0_deg; //-90
  
 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ICSparkFlex _climberMotor{canid::CLIMBER_MOTOR, 60_A}; // _A is amps; 20 amps may be too low for the climber to function

  frc::sim::SingleJointedArmSim _climberSim{frc::DCMotor::NEO(1), GEAR_RATIO, frc::sim::SingleJointedArmSim::EstimateMOI(ARM_LENGTH, ARM_MASS), ARM_LENGTH, ARM_MIN_ANGLE, ARM_MAX_ANGLE, false, ARM_HOME_ANGLE};

  rev::spark::SparkBaseConfig _climberMotorConfig;
  
  frc::Mechanism2d _climberMech{1, 1};  // canvas width and height
  frc::MechanismRoot2d* _climberMechRoot = _climberMech.GetRoot("climberRoot", 0.7, 0.1);  // root x and y
  frc::MechanismLigament2d* _climberMechMountBeam =
    _climberMechRoot->Append<frc::MechanismLigament2d>("mountBeam", 0.25, 90_deg);
  MechanismCircle2d _climberMechGear{_climberMechMountBeam, "gear", 0.075, 90_deg};
  frc::MechanismLigament2d* _climberMechArmLig1 =
    _climberMechMountBeam->Append<frc::MechanismLigament2d>("armLigament1", 0.23, 90_deg);
  frc::MechanismLigament2d* _climberMechArmLig2 =
    _climberMechArmLig1->Append<frc::MechanismLigament2d>("armLigament2", 0.25, 90_deg);   

};