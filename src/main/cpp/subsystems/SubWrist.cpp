// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubWrist.h"

#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

SubWrist::SubWrist() {
    _wristMotorConfig.encoder
        .PositionConversionFactor(1 / WRIST_GEARING)
        .VelocityConversionFactor(WRIST_GEARING / 60);
    _wristMotor.AdjustConfig(_wristMotorConfig);

    _wristMotor.SetFeedbackGains(P, I, D);
    _wristMotor.SetFeedforwardGains(S, G, true, V, A);

    frc::SmartDashboard::PutData("Wrist/Wrist Mechanism2d", &_wristMech);
    frc::SmartDashboard::PutNumber("Wrist/Estimated MOI", WRIST_MOI.value());
    frc::SmartDashboard::PutData("Wrist/Wrist Motor", (wpi::Sendable*)&_wristMotor);
}

// This method will be called once per scheduler run
void SubWrist::Periodic() {
    units::degree_t wristAngle = _wristMotor.GetPosition();
    frc::SmartDashboard::PutNumber("Wrist/Motor Degrees", wristAngle.value());
    _wristMechArmLig->SetAngle(wristAngle);
}

void SubWrist::SimulationPeriodic() {
    _wristSim.SetInputVoltage(_wristMotor.CalcSimVoltage());
    _wristSim.Update(20_ms);
    _wristMotor.IterateSim(_wristSim.GetVelocity());
}

void SubWrist::SetAngle(units::degree_t angle){
    if( (angle >= MIN_ANGLE) && (angle <= MAX_ANGLE) ) {
        _wristMotor.SetPositionTarget(angle);
    } 
}

frc2::CommandPtr SubWrist::SetWristAngle(units::degree_t angle) {
    return frc2::cmd::RunOnce([this, angle] { SetAngle(angle); });
}