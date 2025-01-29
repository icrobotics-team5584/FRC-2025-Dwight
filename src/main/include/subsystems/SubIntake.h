// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "utilities/ICSparkFlex.h"
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
  ICSparkFlex _intakeMotor{canid::IntakeMotor, 30_A};
  ctre::phoenix6::configs::TalonFXConfiguration _configIntakePivotMotor{};
  ICSparkFlex _intakePivotMotor{canid::IntakePivotMotor, 30_A};

  //Intake arm mechanism sim and mechanism2d
  frc::sim::SingleJointedArmSim _intakeSim{frc::DCMotor::Falcon500(1), ARM_GEARING, frc::sim::SingleJointedArmSim::EstimateMOI(ARM_LENGTH, ARM_MASS), ARM_LENGTH, ARM_MIN_ANGLE, ARM_MAX_ANGLE, false, ARM_HOME_ANGLE};
  
  frc::Mechanism2d _singleJointedArmMech{3, 3};  // canvas width and height
  frc::MechanismRoot2d* _armRoot = _singleJointedArmMech.GetRoot("armRoot", 1, 1);  // root x and y
  frc::MechanismLigament2d* _arm1Ligament =
    _armRoot->Append<frc::MechanismLigament2d>("ligament2", ARM_LENGTH.value(), 0_deg);

  //Intake rollers sim and mechanism2d
  static constexpr double ROLLER_GEARING = 1;
  static constexpr units::kilogram_square_meter_t ROLLER_MOI = 0.001_kg_sq_m;
  frc::sim::DCMotorSim _rollerSim{frc::LinearSystemId::DCMotorSystem(frc::DCMotor::NeoVortex(), ROLLER_MOI, ROLLER_GEARING), frc::DCMotor::NeoVortex()};

  frc::Mechanism2d _rollerMech{6, 4};
  frc::MechanismRoot2d* _rollerMechRoot = _rollerMech.GetRoot("rollerRoot", 1, 2);
  MechanismCircle2d _rollerMechLeft{_rollerMechRoot, "leftRoller", 0.75, 0_deg, 36, 10};
  frc::MechanismLigament2d* _rollerMechConnectorLig = 
    _rollerMechRoot->Append<frc::MechanismLigament2d>("connector", 3, 0_deg, 0);
  MechanismCircle2d _rollerMechRight{_rollerMechConnectorLig, "rightRoller", 0.75, 0_deg, 36, 10};
  
  nt::GenericEntry* _armXOffset;
  nt::GenericEntry* _armYOffset;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
