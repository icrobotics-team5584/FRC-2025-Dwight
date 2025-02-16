// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubElevator.h"
#include <frc2/command/Commands.h>
#include <frc/simulation/ElevatorSim.h>
#include <units/angle.h>
#include <math.h>
#include "utilities/RobotLogs.h"
#include <units/angular_velocity.h>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>
#include <utilities/RobotLogs.h>
#include <ctre/phoenix6/configs/Configs.hpp>

using namespace ctre::phoenix6;

SubElevator::SubElevator() {
  // PID Gains for Motion Magic
  _motorConfig.Slot0.kP = _P;
  _motorConfig.Slot0.kI = _I;
  _motorConfig.Slot0.kD = _D;
  _motorConfig.Slot0.kV = _V;
  _motorConfig.Slot0.kA = _A;
  _motorConfig.Slot0.kG = _G;

  // Voltage Configuration
  _motorConfig.Voltage.PeakForwardVoltage = 0_V;
  _motorConfig.Voltage.PeakReverseVoltage = -0_V;

  // Brake Mode
  _motorConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

  // invert motors
  _motorConfig.MotorOutput.Inverted = true;

    // Current Limits
    _motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    _motorConfig.CurrentLimits.SupplyCurrentLowerLimit = 20.0_A;
    _motorConfig.CurrentLimits.SupplyCurrentLimit = 60.0_A;
    _motorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5_s;
    _motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    _motorConfig.CurrentLimits.StatorCurrentLimit = 80.0_A;
    _motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    _motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    _motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (_MAX_HEIGHT/_DRUM_CIRCUMFERENCE).value() * 1_tr;
    _motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0_tr;

    // Feedback Sensor Ratio
    _motorConfig.Feedback.SensorToMechanismRatio = _GEAR_RATIO;

  // Motion Magic ConfigurationS
  _motorConfig.MotionMagic.MotionMagicCruiseVelocity =
      _CRUISE_VELOCITY.value() / _DRUM_CIRCUMFERENCE.value() * 1_tr / 1_s;  // Adjust
  _motorConfig.MotionMagic.MotionMagicAcceleration =
      _ACCELERATION.value() / _DRUM_CIRCUMFERENCE.value() * 1_tr / 1_s / 1_s;  // Adjust

  _elevatorMotor1.GetConfigurator().Apply(_motorConfig);
  _elevatorMotor2.GetConfigurator().Apply(_motorConfig);

  // Set motor 2 to follow motor 1
  _elevatorMotor2.SetControl(controls::Follower(_elevatorMotor1.GetDeviceID(), false));
  _elevatorMotor1.GetClosedLoopReference().SetUpdateFrequency(100_Hz);

}

frc2::CommandPtr SubElevator::CmdElevatorToPosition(units::meter_t height) {
  return RunOnce([this, height] {
    _targetHeight = height;
    if (height < _MIN_HEIGHT) {
      _elevatorMotor1.SetControl(
          controls::MotionMagicVoltage(RotationsFromHeight(_MIN_HEIGHT)).WithEnableFOC(true));
    }

    if (height > _L4_HEIGHT) {
      _elevatorMotor1.SetControl(
          controls::MotionMagicVoltage(RotationsFromHeight(_L4_HEIGHT)).WithEnableFOC(true));
    }

    else {
      _elevatorMotor1.SetControl(
          controls::MotionMagicVoltage(RotationsFromHeight(height)).WithEnableFOC(true));
    }
  });
}

frc2::CommandPtr SubElevator::CmdSetL1() {
  return CmdElevatorToPosition(_L1_HEIGHT);
}

frc2::CommandPtr SubElevator::CmdSetL2() {
  return CmdElevatorToPosition(_L2_HEIGHT);
}

frc2::CommandPtr SubElevator::CmdSetL3() {
  return CmdElevatorToPosition(_L3_HEIGHT);
}

frc2::CommandPtr SubElevator::CmdSetL4() {
  return CmdElevatorToPosition(_L4_HEIGHT);
}

frc2::CommandPtr SubElevator::CmdSetClimb() {
  return CmdElevatorToPosition(_CLIMB_HEIGHT);
}

frc2::CommandPtr SubElevator::CmdSetClearLowAlgea(){
    return CmdElevatorToPosition(_ALGAE_LOW_HEIGHT);
}

frc2::CommandPtr SubElevator::CmdSetClearHighAlgea(){
    return CmdElevatorToPosition(_ALGAE_HIGH_HEIGHT);
}

frc2::CommandPtr SubElevator::CmdSetSource(){
    return CmdElevatorToPosition(_SOURCE_HEIGHT);
}

units::turn_t SubElevator::RotationsFromHeight(units::meter_t height) {
  return height.value() / _DRUM_CIRCUMFERENCE.value() * 1_tr;
};

units::meter_t SubElevator::HeightFromRotations(units::turn_t turns) {
  return turns.value() * _DRUM_CIRCUMFERENCE.value() * 1_m;
};

units::turns_per_second_t SubElevator::RotationsFromMetersPerSecond(
    units::meters_per_second_t meterspersec) {
  return meterspersec.value() / _DRUM_CIRCUMFERENCE.value() * 1_tps;
};

// Reset motor position to 0
frc2::CommandPtr SubElevator::ZeroElevator() {
  return RunOnce([this] {
    _elevatorMotor1.SetPosition(RotationsFromHeight(_MIN_HEIGHT));
    _elevatorMotor2.SetPosition(RotationsFromHeight(_MIN_HEIGHT));
  });
}

// Get motor1 current
units::ampere_t SubElevator::GetM1Current() {
  return _elevatorMotor1.GetStatorCurrent().GetValue();
}

units::meter_t SubElevator::GetTargetHeight() {
  return _targetHeight;
}

void SubElevator::EnableSoftLimit(bool enabled) {
  // Configure the forward soft limit for elevatorMotor1
  if (!enabled) {
    _motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    _motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    _elevatorMotor1.GetConfigurator().Apply(_motorConfig);
    _elevatorMotor2.GetConfigurator().Apply(_motorConfig);
  } else {
    _motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    _motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    _elevatorMotor1.GetConfigurator().Apply(_motorConfig);
    _elevatorMotor2.GetConfigurator().Apply(_motorConfig);
  }

  // Configure the forward soft limit for elevatorMotor1
  _motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
  _elevatorMotor1.GetConfigurator().Apply(_motorConfig);

  // Configure the reverse soft limit for elevatorMotor2
  _elevatorMotor1.GetConfigurator().Apply(_motorConfig);
}

frc2::CommandPtr SubElevator::ElevatorToClimbHeight() {
  return CmdElevatorToPosition(0.14_m);
}

frc2::CommandPtr SubElevator::Climb() {
  return frc2::cmd::Run([this] {
           _elevatorMotor1.SetControl(
               ctre::phoenix6::controls::VoltageOut(Logger::Tune("ElevatorClimbVoltage", 0_V)));
         })
      .Until([this] {
        return HeightFromRotations(_elevatorMotor1.GetPosition(true).GetValue()) < 0.02_m;
      })
      .FinallyDo([this] {
        auto targHeight = HeightFromRotations(_elevatorMotor1.GetPosition(true).GetValue());
        _elevatorMotor1.SetControl(
            controls::PositionVoltage(RotationsFromHeight(targHeight)).WithEnableFOC(true));
      });
}

frc2::CommandPtr SubElevator::ManualElevatorMovementUP() {
  return frc2::cmd::RunEnd(
      [this] {
        auto currentHeight = HeightFromRotations(_elevatorMotor1.GetPosition(true).GetValue());
        if(currentHeight < _L4_HEIGHT){
            _elevatorMotor1.SetControl(ctre::phoenix6::controls::VoltageOut(1_V));}
        else{
            _elevatorMotor1.StopMotor();
        }},
      [this] {
        auto targHeight = HeightFromRotations(_elevatorMotor1.GetPosition(true).GetValue());
        _elevatorMotor1.SetControl(
            controls::PositionVoltage(RotationsFromHeight(targHeight)).WithEnableFOC(true));
      });
}

frc2::CommandPtr SubElevator::ManualElevatorMovementDOWN() {
  return frc2::cmd::RunEnd(
      [this] {
        auto currentHeight = HeightFromRotations(_elevatorMotor1.GetPosition(true).GetValue());
        if(currentHeight > _MIN_HEIGHT + 0.1_m){
            _elevatorMotor1.SetControl(ctre::phoenix6::controls::VoltageOut(-0.3_V));}
        else{
            _elevatorMotor1.StopMotor();
        }},
      [this] {
        auto targHeight = HeightFromRotations(_elevatorMotor1.GetPosition(true).GetValue());
        _elevatorMotor1.SetControl(
            controls::PositionVoltage(RotationsFromHeight(targHeight)).WithEnableFOC(true));
      });
}

frc2::CommandPtr SubElevator::ManualElevatorMovementAlgae() {
  return frc2::cmd::StartEnd(
      [this] { _elevatorMotor1.SetControl(ctre::phoenix6::controls::VoltageOut(-0.3_V)); },
      [this] {
        auto targHeight = HeightFromRotations(_elevatorMotor1.GetPosition(true).GetValue());
        _elevatorMotor1.SetControl(
            controls::PositionVoltage(RotationsFromHeight(targHeight)).WithEnableFOC(true));
      });
    }

frc2::CommandPtr SubElevator::DriveDownToReset() {
  return frc2::cmd::RunOnce([this] {_elevatorMotor1.SetControl(ctre::phoenix6::controls::VoltageOut(-1_V));});
    }

//Check if elevator has touched the bottom
frc2::CommandPtr SubElevator::ElevatorResetCheck() {
    return frc2::cmd::RunOnce ([this] {ResetM1 = false;})
    .AndThen(
    frc2::cmd::Run([this] {
        
        if (GetM1Current() > zeroingCurrentLimit) {
            ResetM1 = true;
        }
        else {
            Reseting = false;
        }
    }).Until([this] { return ResetM1; }));
}

// Ptr cmd of Stop()
frc2::CommandPtr SubElevator::ElevatorStop() {
  return frc2::cmd::RunOnce([this] { SubElevator::GetInstance().Stop(); });
}

bool SubElevator::IsAtTarget() {
  units::meter_t currentHeight = HeightFromRotations(_elevatorMotor1.GetPosition().GetValue());
  units::meter_t tolerance = 0.05_m;
  if (currentHeight > _targetHeight - tolerance && currentHeight < _targetHeight + tolerance) {
    return true;
  }

  else {
    return false;
  }
}

void SubElevator::SetMotorVoltageLimits12V() {
  _motorConfig.Voltage.PeakForwardVoltage = 12_V;
  _motorConfig.Voltage.PeakReverseVoltage = 12_V;
  _elevatorMotor1.GetConfigurator().Apply(_motorConfig);
  _elevatorMotor2.GetConfigurator().Apply(_motorConfig);
}

void SubElevator::CheckAndChangeCurrentLimitIfReset() {
  if (ResetM1 == false) {
    _motorConfig.Voltage.PeakForwardVoltage = 0_V;
    _motorConfig.Voltage.PeakReverseVoltage = 0_V;
    _elevatorMotor1.GetConfigurator().Apply(_motorConfig);
    _elevatorMotor2.GetConfigurator().Apply(_motorConfig);
  }
}
// Auto elevator reset by bringing elevator to zero position then reset (can be used in tele-op)
frc2::CommandPtr SubElevator::ElevatorAutoReset() {
    return frc2::cmd::RunOnce([this] { Reseting = true; EnableSoftLimit(false);})
        .AndThen([this] {SetMotorVoltageLimits12V();})
        .AndThen(DriveDownToReset())
        .AndThen(ElevatorResetCheck())
        .AndThen(ZeroElevator())
        .AndThen([this] {Stop();})
        .FinallyDo([this] {EnableSoftLimit(true); CheckAndChangeCurrentLimitIfReset();} );
        }

// Stop motor
void SubElevator::Stop() {
  _elevatorMotor1.StopMotor();
}

void SubElevator::SetBrakeMode(bool mode) {
  if (mode == true) {
    _elevatorMotor1.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
    _elevatorMotor2.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Brake);
  } else if (mode == false) {
    _elevatorMotor1.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
    _elevatorMotor2.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
  }
}

// This method will be called once per scheduler run
void SubElevator::Periodic() {
  Logger::LogFalcon("Elevator/Motor1", _elevatorMotor1);

  Logger::LogFalcon("Elevator/Motor2", _elevatorMotor2);
  Logger::Log("Elevator/Motor1/Height",
              HeightFromRotations(_elevatorMotor1.GetPosition().GetValue()));
  Logger::Log("Elevator/Motor2/Height",
              HeightFromRotations(_elevatorMotor2.GetPosition().GetValue()));
  Logger::Log("Elevator/Motor1/Target",
              HeightFromRotations(_elevatorMotor1.GetClosedLoopReference().GetValue() * 1_tr));
  Logger::Log("Elevator/Motor2/Target",
              HeightFromRotations(_elevatorMotor2.GetClosedLoopReference().GetValue() * 1_tr));
  Logger::Log("Elevator/_targetheight", _targetHeight);
  Logger::Log("Elevator/M1Current", GetM1Current().value());
  Logger::Log("Elevator/ResetM1", ResetM1);
}

void SubElevator::SimulationPeriodic() {
  auto& motorState = _elevatorMotor1.GetSimState();
  _motorSim.SetInputVoltage(motorState.GetMotorVoltage());
  _motorSim.Update(20_ms);
  _motorSim.GetVelocity();
  motorState.SetRawRotorPosition(_GEAR_RATIO * RotationsFromHeight(_motorSim.GetPosition()));
  motorState.SetRotorVelocity(_GEAR_RATIO * RotationsFromMetersPerSecond(_motorSim.GetVelocity()));
  _motorSim.GetCurrentDraw();
};
