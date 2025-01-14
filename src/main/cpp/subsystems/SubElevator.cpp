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

using namespace ctre::phoenix6;

SubElevator::SubElevator() {
    ctre::phoenix6::configs::TalonFXConfiguration MotorConfig{};
    
    // PID Gains for Motion Magic
    MotorConfig.Slot0.kP = _P;
    MotorConfig.Slot0.kI = _I;
    MotorConfig.Slot0.kD = _D;
    MotorConfig.Slot0.kV = _V;
    MotorConfig.Slot0.kA = _A;
    MotorConfig.Slot0.kG = _G;

    // Voltage Configuration
    MotorConfig.Voltage.PeakForwardVoltage = 12_V;
    MotorConfig.Voltage.PeakReverseVoltage = -12_V;

    // invert motors
    MotorConfig.MotorOutput.Inverted = true;

    // Current Limits
    MotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    MotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 20.0_A; //40
    MotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0_A; //80
    MotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5_s;
    MotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    MotorConfig.CurrentLimits.StatorCurrentLimit = 30.0_A;//30
    MotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    MotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    MotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (_L4_HEIGHT/_DRUM_CIRCUMFERENCE).value() * 1_tr;
    MotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0_tr;

    // Feedback Sensor Ratio
    MotorConfig.Feedback.SensorToMechanismRatio = 14;

    // Motion Magic ConfigurationS
    MotorConfig.MotionMagic.MotionMagicCruiseVelocity = _CRUISE_VELOCITY.value() / _DRUM_CIRCUMFERENCE.value() * 1_tr / 1_s; // Adjust
    MotorConfig.MotionMagic.MotionMagicAcceleration = _ACCELERATION.value() / _DRUM_CIRCUMFERENCE.value() * 1_tr / 1_s / 1_s; // Adjust

    _elevatorMotor1.GetConfigurator().Apply(MotorConfig);
    _elevatorMotor2.GetConfigurator().Apply(MotorConfig);

    // Set motor 2 to follow motor 1
    _elevatorMotor2.SetControl(controls::Follower(_elevatorMotor1.GetDeviceID(), false));
    _elevatorMotor1.GetClosedLoopReference().SetUpdateFrequency(100_Hz);
}

frc2::CommandPtr SubElevator::CmdElevatorToPosition(units::meter_t height){
    return RunOnce([this, height]{
    if(height < _MIN_HEIGHT){
       _elevatorMotor1.SetControl(controls::MotionMagicVoltage(RotationsFromHeight(_MIN_HEIGHT)).WithEnableFOC(true));
        }

    if(height > _L4_HEIGHT){
        _elevatorMotor1.SetControl(controls::MotionMagicVoltage(RotationsFromHeight(_L4_HEIGHT)).WithEnableFOC(true));
    }

    else {
         _elevatorMotor1.SetControl(controls::MotionMagicVoltage(RotationsFromHeight(height)).WithEnableFOC(true));
    }
    });}

frc2::CommandPtr SubElevator::CmdSetL1(){
    return CmdElevatorToPosition(_L1_HEIGHT);
    }

frc2::CommandPtr SubElevator::CmdSetL2(){
    return CmdElevatorToPosition(_L2_HEIGHT);
    }

frc2::CommandPtr SubElevator::CmdSetL3(){
    return CmdElevatorToPosition(_L3_HEIGHT);
    }

frc2::CommandPtr SubElevator::CmdSetL4(){
    return CmdElevatorToPosition(_L4_HEIGHT);
    }

frc2::CommandPtr SubElevator::CmdSetSource(){
    return CmdElevatorToPosition(_SOURCE_HEIGHT);
    }


units::turn_t SubElevator::RotationsFromHeight(units::meter_t height){
    return height.value() / _DRUM_CIRCUMFERENCE.value() * 1_tr;
};

units::meter_t SubElevator::HeightFromRotations(units::turn_t turns) {
    return turns.value() * _DRUM_CIRCUMFERENCE.value() * 1_m;
}

units::turns_per_second_t SubElevator::RotationsFromMetersPerSecond(units::meters_per_second_t meterspersec){
    return meterspersec.value() / _DRUM_CIRCUMFERENCE.value() * 1_tps;
};

//Reset motor position to 0
frc2::CommandPtr SubElevator::ZeroElevator() {
    return RunOnce([this]{
    _elevatorMotor1.SetPosition(RotationsFromHeight(_MIN_HEIGHT));
    _elevatorMotor2.SetPosition(RotationsFromHeight(_MIN_HEIGHT));
   });
}

//Get motor1 current
units::ampere_t SubElevator::GetM1Current() {
    return _elevatorMotor1.GetStatorCurrent().GetValue();
}


void SubElevator::EnableSoftLimit(bool enabled) {
    ctre::phoenix6::configs::TalonFXConfiguration MotorConfig{};
    // Configure the forward soft limit for elevatorMotor1
    MotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    _elevatorMotor1.GetConfigurator().Apply(MotorConfig);
      

    // Configure the reverse soft limit for elevatorMotor2
    _elevatorMotor1.GetConfigurator().Apply(MotorConfig);
    
}

frc2::CommandPtr SubElevator::ManualElevatorMovementUP() {
  return frc2::cmd::StartEnd(
      [this] { _elevatorMotor1.SetControl(ctre::phoenix6::controls::VoltageOut(4_V)); },
      [this] {
        auto targHeight = HeightFromRotations(_elevatorMotor1.GetPosition(true).GetValue());
        _elevatorMotor1.SetControl(controls::PositionVoltage(RotationsFromHeight(targHeight)).WithEnableFOC(true));
      });
    }

frc2::CommandPtr SubElevator::ManualElevatorMovementDOWN() {
  return frc2::cmd::StartEnd(
      [this] { _elevatorMotor1.SetControl(ctre::phoenix6::controls::VoltageOut(-4_V)); },
      [this] {
        auto targHeight = HeightFromRotations(_elevatorMotor1.GetPosition(true).GetValue());
        _elevatorMotor1.SetControl(controls::PositionVoltage(RotationsFromHeight(targHeight)).WithEnableFOC(true));
      });
    }

frc2::CommandPtr SubElevator::ManualElevatorMovementDOWNSLOW() {
  return frc2::cmd::RunOnce(
      [this] { _elevatorMotor1.SetControl(ctre::phoenix6::controls::VoltageOut(-1_V)); });
    }

//Check if elevator has touched the bottom
frc2::CommandPtr SubElevator::ElevatorResetCheck() {
    return frc2::cmd::RunOnce ([this] {ResetM1 = false;})
    .AndThen(
    frc2::cmd::Run([this] {
        
        if (GetM1Current().value() > zeroingCurrentLimit && !ResetM1) {
            _elevatorMotor1.StopMotor(); ResetM1 = true;
        }
        if (ResetM1) {
            Reseting = false;
        }
    }).Until([this] { return ResetM1; }));
}

//Ptr cmd of Stop()
frc2::CommandPtr SubElevator::ElevatorStop() {
    return frc2::cmd::RunOnce([this] {SubElevator::GetInstance().Stop();});
}

//Auto elevator reset by bringing elevator to zero position then reset (can be used in tele-op)
frc2::CommandPtr SubElevator::ElevatorAutoReset() {
    return frc2::cmd::RunOnce([this] { Reseting = true; EnableSoftLimit(false);})
        .AndThen(ManualElevatorMovementDOWNSLOW())
        .AndThen(ElevatorResetCheck())
        .AndThen(ZeroElevator())
        .AndThen(ElevatorStop())
        .FinallyDo([this] {Reseting = false; Reseted = true; EnableSoftLimit(false); Stop();});
}

//Stop motor
void SubElevator::Stop() {
  _elevatorMotor1.StopMotor();
}

// This method will be called once per scheduler run
void SubElevator::Periodic() {
    Logger::LogFalcon("Elevator/Motor1", _elevatorMotor1);
    Logger::LogFalcon("Elevator/Motor2", _elevatorMotor2);
    Logger::Log("Elevator/Motor1/Height", HeightFromRotations(_elevatorMotor1.GetPosition().GetValue()));
    Logger::Log("Elevator/Motor2/Height", HeightFromRotations(_elevatorMotor2.GetPosition().GetValue()));
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
