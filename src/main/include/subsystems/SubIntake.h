// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/ICSparkMax.h"
#include <frc2/command/SubsystemBase.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include "Constants.h"
#include <frc/DigitalInput.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/simulation/DCMotorSim.h>
#include <frc/system/plant/LinearSystemId.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include "utilities/MechanismCircle2d.h"

class SubIntake : public frc2::SubsystemBase {
 public:
  SubIntake();
  static SubIntake &GetInstance() {
    static SubIntake inst;
    return inst;
  }

  frc2::CommandPtr Intake();
  frc2::CommandPtr Outtake();
  frc2::CommandPtr Deploy();
  frc2::CommandPtr Stow();
  frc2::CommandPtr DeployAndRetract();
  void SetDesiredAngle(units::degree_t angle);

  const units::angle::degree_t INTAKE_MAX_ANGLE = 90_deg;
  const units::angle::degree_t INTAKE_MIN_ANGLE = 0_deg;
  const double P = 10;
  const double I = 0;
  const double D = 1;

  static constexpr double ARM_GEARING = 15;
  const units::length::meter_t ARM_LENGTH = 0.3_m;
  const units::mass::kilogram_t ARM_MASS = 6_kg;
  const units::angle::degree_t ARM_MIN_ANGLE = 0_deg;
  const units::angle::degree_t ARM_MAX_ANGLE = 90_deg;
  const units::angle::degree_t ARM_HOME_ANGLE = 45_deg;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  ICSparkMax _intakeRollerMotor{canid::IntakeRollerMotor, 30_A};
  ctre::phoenix6::configs::TalonFXConfiguration _configIntakePivotMotor{};
  ctre::phoenix6::hardware::TalonFX _intakePivotMotor{canid::IntakePivotMotor};

  //Intake sims and mechanism2d
  frc::sim::SingleJointedArmSim _intakeSim{frc::DCMotor::Falcon500(1), ARM_GEARING, frc::sim::SingleJointedArmSim::EstimateMOI(ARM_LENGTH, ARM_MASS), ARM_LENGTH, ARM_MIN_ANGLE, ARM_MAX_ANGLE, false, ARM_HOME_ANGLE};
  
  static constexpr double ROLLER_GEARING = 1; //need correct gearing for intake motor
  static constexpr units::kilogram_square_meter_t ROLLER_MOI = 0.001_kg_sq_m; //need correct moment of inertia for intake motor
  frc::sim::DCMotorSim _rollerSim{frc::LinearSystemId::DCMotorSystem(frc::DCMotor::NeoVortex(), ROLLER_MOI, ROLLER_GEARING), frc::DCMotor::NeoVortex()};

  frc::Mechanism2d _intakeMech{6, 4};  // canvas width and height
  frc::MechanismRoot2d* _intakeMechRoot = _intakeMech.GetRoot("intakeRoot", 1, 1);  // root x and y
  frc::MechanismLigament2d* _intakeMechArmLig1 =
    _intakeMechRoot->Append<frc::MechanismLigament2d>("armLigament1", 2, 0_deg);
  // MechanismCircle2d _intakeMechTopRoller{_intakeMechArmLig1, "topRoller", 0.75, 0_deg, 36, 10};
  // frc::MechanismLigament2d* _intakeMechArmLig2 = 
  //   _intakeMechRoot->Append<frc::MechanismLigament2d>("armLigament2", 3, 90_deg, 0);
  // MechanismCircle2d _intakeMechBottomRoller{_intakeMechArmLig2, "bottomRoller", 0.75, 180_deg, 36, 10};
  
  nt::GenericEntry* _armXOffset;
  nt::GenericEntry* _armYOffset;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
