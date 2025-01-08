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
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>

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
  void SetDesiredAngle(units::degree_t angle);

  const double P = 10;
  const double I = 0;
  const double D = 1;
  const double GearRatio = 15;
  const units::length::meter_t ArmLength = 0.3_m;
  const units::mass::kilogram_t ArmMass = 6_kg;
  const units::angle::degree_t ArmMinAngle = 0_deg;
  const units::angle::degree_t ArmMaxAngle = 90_deg;
  const units::angle::degree_t ArmHomeAngle = 45_deg;

  


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

 private:
 ICSparkMax _intakeLeftMotor{canid::IntakeLeftMotor, 30_A};
 ICSparkMax _intakeRightMotor{canid::IntakeRightMotor, 30_A};

 ctre::phoenix6::configs::TalonFXConfiguration _configIntakePivotMotor{};
 ctre::phoenix6::hardware::TalonFX _intakePivotMotor{canid::IntakePivotMotor};

 // sim stuff
  frc::sim::SingleJointedArmSim _intakeSim{frc::DCMotor::Falcon500(1), GearRatio, frc::sim::SingleJointedArmSim::EstimateMOI(ArmLength, ArmMass), ArmLength, ArmMinAngle, ArmMaxAngle, false, ArmHomeAngle};
 
  frc::Mechanism2d _singleJointedArmMech{3, 3};  // canvas width and height
  frc::MechanismRoot2d* _armRoot = _singleJointedArmMech.GetRoot("armRoot", 1, 1);  // root x and y
  frc::MechanismLigament2d* _arm1Ligament =
      _armRoot->Append<frc::MechanismLigament2d>("ligament2", ArmLength.value(), 0_deg);

  nt::GenericEntry* _armXOffset;
  nt::GenericEntry* _armYOffset;
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
