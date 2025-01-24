// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "utilities/ICSparkMax.h"
#include "Constants.h"
#include <frc/simulation/SingleJointedArmSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>

class SubWrist : public frc2::SubsystemBase {
 public:
  SubWrist();
  static SubWrist &GetInstance() {
    static SubWrist inst;
    return inst;
  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;
  void SimulationPeriodic() override;

  void SetAngle(units::degree_t angle);
  frc2::CommandPtr SetWristAngle(units::degree_t angle);

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  ICSparkMax _wristMotor{canid::WristMotor, 30_A};
  rev::spark::SparkBaseConfig _wristMotorConfig;

  //CONSTANTS
  static constexpr units::degree_t MIN_ANGLE = -360_deg;
  static constexpr units::degree_t MAX_ANGLE = 360_deg;
  static constexpr units::degree_t INITIAL_ANGLE = 0_deg;

  //note: end effector is part of the wrist mechanism, so include it in these values
  static constexpr units::meter_t WRIST_LENGTH = 0.4_m;
  static constexpr units::kilogram_t WRIST_MASS = 3_kg;
  static constexpr units::kilogram_square_meter_t WRIST_MOI = frc::sim::SingleJointedArmSim::EstimateMOI(WRIST_LENGTH, WRIST_MASS);
  static constexpr double WRIST_GEARING = 90; //reduction 90:1

  //sim
  frc::sim::SingleJointedArmSim  _wristSim{frc::DCMotor::NeoVortex(), WRIST_GEARING, WRIST_MOI, WRIST_LENGTH, MIN_ANGLE, MAX_ANGLE, true, INITIAL_ANGLE};
  
  frc::Mechanism2d _wristMech{3, 3};
  frc::MechanismRoot2d* _wristMechRoot = _wristMech.GetRoot("wristRoot", 1, 1);
  frc::MechanismLigament2d* _wristMechArmLig =
    _wristMechRoot->Append<frc::MechanismLigament2d>("wristArm", 1, INITIAL_ANGLE);
};