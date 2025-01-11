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

    // Current Limits
    MotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    MotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40.0_A;
    MotorConfig.CurrentLimits.SupplyCurrentLimit = 80.0_A;
    MotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.5_s;
    MotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    MotorConfig.CurrentLimits.StatorCurrentLimit = 30.0_A;


    // Feedback Sensor Ratio
    MotorConfig.Feedback.SensorToMechanismRatio = 14;

    // Motion Magic ConfigurationS
    MotorConfig.MotionMagic.MotionMagicCruiseVelocity = _CRUISE_VELOCITY.value() / _DRUM_CIRCUMFERENCE.value() * 1_tr / 1_s; // Adjust
    MotorConfig.MotionMagic.MotionMagicAcceleration = _ACCELERATION.value() / _DRUM_CIRCUMFERENCE.value() * 1_tr / 1_s / 1_s; // Adjust

    _ElevatorMotor1.GetConfigurator().Apply(MotorConfig);
    MotorConfig.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    _ElevatorMotor2.GetConfigurator().Apply(MotorConfig);

    // Set motor 2 to follow motor 1
    _ElevatorMotor2.SetControl(controls::Follower(_ElevatorMotor1.GetDeviceID(), false));
}

frc2::CommandPtr SubElevator::CmdElevatorToPosition(units::meter_t height){
    return RunOnce([this, height]{
    if(height < 0.432_m){
       _ElevatorMotor1.SetControl(controls::MotionMagicVoltage(RotationsFromHeight(0.432_m)).WithEnableFOC(true));
        }
    else {
         _ElevatorMotor1.SetControl(controls::MotionMagicVoltage(RotationsFromHeight(height)).WithEnableFOC(true));
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

units::turns_per_second_t SubElevator::RotationsFromMetersPerSecond(units::meters_per_second_t meterspersec){
    return meterspersec.value() / _DRUM_CIRCUMFERENCE.value() * 1_tps;
};

// This method will be called once per scheduler run
void SubElevator::Periodic() {
    Logger::LogFalcon("Elevator/Motor1", _ElevatorMotor1);
    Logger::LogFalcon("Elevator/Motor2", _ElevatorMotor2);
}

void SubElevator::SimulationPeriodic() {
    auto& motorState = _ElevatorMotor1.GetSimState();
    _motorSim.SetInputVoltage(motorState.GetMotorVoltage());
    _motorSim.Update(20_ms);
    _motorSim.GetVelocity();
    motorState.SetRawRotorPosition(_GEAR_RATIO * RotationsFromHeight(_motorSim.GetPosition()));
    motorState.SetRotorVelocity(_GEAR_RATIO * RotationsFromMetersPerSecond(_motorSim.GetVelocity()));
    _motorSim.GetCurrentDraw();
};
