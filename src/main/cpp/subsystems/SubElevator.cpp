// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubElevator.h"
#include <frc2/command/Commands.h>
#include <frc/simulation/ElevatorSim.h>
#include <units/angle.h>
#include <units/constants.h>

using namespace ctre::phoenix6;

SubElevator::SubElevator(){
    ctre::phoenix6::configs::TalonFXConfiguration MotorConfig{};
    MotorConfig.Slot0.kP = _P;
    MotorConfig.Slot0.kI = _I;
    MotorConfig.Slot0.kD = _D;
    MotorConfig.Slot0.kV = _V;
    MotorConfig.Voltage.PeakForwardVoltage = 12_V;
    MotorConfig.Voltage.PeakReverseVoltage = -12_V;
    MotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    MotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40.0_A;
    MotorConfig.CurrentLimits.SupplyCurrentLimit = 50.0_A;
    MotorConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1_s;
    MotorConfig.Feedback.SensorToMechanismRatio = 20;

    _ElevatorMotor1.GetConfigurator().Apply(MotorConfig);
    MotorConfig.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;
    _ElevatorMotor2.GetConfigurator().Apply(MotorConfig);
    _ElevatorMotor2.SetControl(controls::Follower(_ElevatorMotor1.GetDeviceID(), false));
}

frc2::CommandPtr SubElevator::CmdSetL1(){
    return RunOnce([this]{
        _ElevatorMotor1.SetControl(controls::PositionVoltage(5_tr));
    });}

frc2::CommandPtr SubElevator::CmdSetL2(){
    return RunOnce([this]{
        _ElevatorMotor1.SetControl(controls::PositionVoltage(8_tr));
    });}

frc2::CommandPtr SubElevator::CmdSetL3(){
    return RunOnce([this]{
        _ElevatorMotor1.SetControl(controls::PositionVoltage(10_tr));
    });}

frc2::CommandPtr SubElevator::CmdSetL4(){
    return RunOnce([this]{
        _ElevatorMotor1.SetControl(controls::PositionVoltage(13_tr));
    });}

frc2::CommandPtr SubElevator::CmdSetSource(){
    return RunOnce([this]{

    });}


units::turn_t SubElevator::RotationsFromHeight(units::meter_t height){
    return (height / (_DRUM_RADIUS * 2 * units::constants::pi)).value() * 1_tr;
};



// This method will be called once per scheduler run
void SubElevator::Periodic() {}



void SubElevator::SimulationPeriodic() {
    auto& motorState = _ElevatorMotor1.GetSimState();
    _motorSim.SetInputVoltage(motorState.GetMotorVoltage());
    _motorSim.Update(20_ms);
    motorState.SetRawRotorPosition(_GEAR_RATIO * RotationsFromHeight(_motorSim.GetPosition()));
};
