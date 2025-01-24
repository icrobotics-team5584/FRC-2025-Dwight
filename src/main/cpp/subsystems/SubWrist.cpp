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

    _wristMotor.SetFeedbackGains(0, 0, 0);
    _wristMotor.SetFeedforwardGains(0_V, 0_V, true, (0_V / 1_tps), (0_V / 1_tr_per_s_sq));

    frc::SmartDashboard::PutData("Wrist/Wrist Mechanism2d", &_wristMech);
    frc::SmartDashboard::PutData("Wrist/Wrist Motor", (wpi::Sendable*)&_wristMotor);
}

// This method will be called once per scheduler run
void SubWrist::Periodic() {
    _wristMechArmLig->SetAngle(_wristMotor.GetPosition());
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