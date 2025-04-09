// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubClimber.h"

#include <units/voltage.h>
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc/RobotBase.h"

SubClimber::SubClimber() {
  frc::SmartDashboard::PutData("Climber/Motor", &_climberMotor);

  _climberMotorConfig.encoder.PositionConversionFactor(1 / GEAR_RATIO);
  _climberMotorConfig.encoder.VelocityConversionFactor(GEAR_RATIO / 60.0);
  _climberMotorConfig.closedLoop.Pid(P, I, D, rev::spark::ClosedLoopSlot::kSlot0);
  //_climberMotorConfig.closedLoop.MinOutput(-0.5);  // limits climb speed
  _climberMotorConfig.Inverted(true).SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
  auto err = _climberMotor.AdjustConfig(_climberMotorConfig);
  frc::SmartDashboard::PutNumber("Climber/config set err", (int)err);
}

// This method will be called once per scheduler run
void SubClimber::Periodic() {
  // clang-format off
  frc::SmartDashboard::PutNumber("Climber/MotorTarget", _climberMotor.GetPositionTarget().value());
  frc::SmartDashboard::PutData("Climber/armMechDisplay", &_climberMech);
  frc::SmartDashboard::PutNumber("Climber/MotorVoltage", _climberMotor.GetMotorVoltage().value());
  frc::SmartDashboard::PutNumber("Climber/Current", _climberMotor.GetOutputCurrent());
  frc::SmartDashboard::PutNumber("Climber/PositionMotor", (_climberMotor.GetPosition().value()));
  frc::SmartDashboard::PutNumber("Climber/PositionArm", _climberMotor.GetPosition().value() / GEAR_RATIO);
  frc::SmartDashboard::PutNumber("Climber/M1Current", GetM1Current().value());
  frc::SmartDashboard::PutNumber("Climber/hasReset", _hasReset);
  frc::SmartDashboard::PutNumber("Climber/resetting", _resetting);
  // clang-format on

  if (_hasReset == false && _resetting == false) {
    _climberMotor.Set(0);
  }

  units::angle::degree_t pivotAngle = _climberMotor.GetPosition();
  _climberMechGear.SetAngle(pivotAngle+90_deg);
  _climberMechArmLig1->SetAngle(pivotAngle+90_deg);
}

void SubClimber::SetBrakeMode(bool mode) {
  rev::spark::SparkBaseConfig _neutralModeConfig;
  if (mode == true) {
    _neutralModeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    _climberMotor.AdjustConfigNoPersist(_neutralModeConfig);
  } else if (mode == false) {
    _neutralModeConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
    _climberMotor.AdjustConfigNoPersist(_neutralModeConfig);
  }
}

bool SubClimber::IsAtTarget() {
  units::turn_t currentRotation = _climberMotor.GetPosition();
  units::turn_t targetRotation = _climberMotor.GetPositionTarget();
  units::turn_t tolerance = 0.05_deg;
  if (currentRotation > targetRotation - tolerance &&
      currentRotation < targetRotation + tolerance) {
    return true;
  } else {
    return false;
  }
}

bool SubClimber::IsOut() {
  if (_hasReset == false) {return false;}
  auto errorAngle = _climberMotor.GetPosition() - STOW_TURNS;
  return errorAngle < -5_deg || errorAngle > 5_deg;
}

// Auto climber reset by bringing Climber to zero position then reset
frc2::CommandPtr SubClimber::ClimberAutoReset() {
  return RunOnce([this] { _resetting = true; })
      .AndThen(ManualClimberMovementDOWNSLOW())
      .AndThen(ClimberResetCheck())
      .AndThen([this] { _climberMotor.SetPosition(0_deg); })
      .FinallyDo([this] {
        _climberMotor.StopMotor();
        _resetting = false;
      });
}

frc2::CommandPtr SubClimber::ClimberResetCheck() {
  // clang-format off
  return RunOnce([this] { _hasReset = false; })
    .AndThen(Run([this] {
      if (GetM1Current() > zeroingCurrentLimit) {
        _hasReset = true;
      }
      if (frc::RobotBase::IsSimulation() == true) {
        _hasReset = true;
      }
    }).Until([this] { return _hasReset; }));
  // clang-format on
}

frc2::CommandPtr SubClimber::ManualClimberMovementUP() {
  return StartEnd([this] { _climberMotor.SetVoltage(4_V); },
                  [this] {
                    auto targRot = _climberMotor.GetPosition();
                    _climberMotor.SetMaxMotionTarget(targRot);
                  });
}

frc2::CommandPtr SubClimber::ManualClimberMovementDOWN() {
  return StartEnd([this] { _climberMotor.SetVoltage(-4_V); },
                  [this] {
                    auto targRot = _climberMotor.GetPosition();
                    _climberMotor.SetMaxMotionTarget(targRot);
                  });
}

frc2::CommandPtr SubClimber::ManualClimberMovementDOWNSLOW() {
  return RunOnce([this] { _climberMotor.SetVoltage(-1_V); });
}

units::ampere_t SubClimber::GetM1Current() {
  return _climberMotor.GetOutputCurrent() * 1_A;
}

frc2::Trigger SubClimber::IsClimbing() {
  return frc2::Trigger([this] { return _climberMotor.GetPosition() > STOW_TURNS + 5_deg; });
}

frc2::CommandPtr SubClimber::StowClimber() {
  return RunOnce([this] { _climberMotor.SetPositionTarget(STOW_TURNS); });
}

frc2::CommandPtr SubClimber::ReadyClimber() {
  return RunOnce([this] { _climberMotor.SetPositionTarget(PREPARE_TURNS); });
}

frc2::CommandPtr SubClimber::ClimbToHalfway() {
  return RunOnce([this] { _climberMotor.SetPositionTarget(HALF_CLIMB_TURNS); });
}

frc2::CommandPtr SubClimber::Climb() {
  return RunOnce([this] { _climberMotor.SetPositionTarget(CLIMB_TURNS); });
}

frc2::CommandPtr SubClimber::set12V() {
  return RunOnce([this] { _climberMotor.SetVoltage(12_V); });
}

void SubClimber::SimulationPeriodic() {
  _climberSim.SetInputVoltage(_climberMotor.CalcSimVoltage());
  _climberSim.Update(20_ms);

  auto armAngle = _climberSim.GetAngle();
  auto armVel = _climberSim.GetVelocity();
  _climberMotor.IterateSim(armVel);

  frc::SmartDashboard::PutNumber("Climber/armAngle", armAngle.value());
  frc::SmartDashboard::PutNumber("Climber/armVelocity", armVel.value());
  frc::SmartDashboard::PutNumber("Climber/SimVoltage", _climberMotor.CalcSimVoltage().value());
}