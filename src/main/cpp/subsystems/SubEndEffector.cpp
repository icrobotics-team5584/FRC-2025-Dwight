// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubEndEffector.h"



SubEndEffector::SubEndEffector() = default;

// This method will be called once per scheduler run
void SubEndEffector::Periodic() {}

frc2::CommandPtr SubEndEffector::FeedUp() {
    return StartEnd([this] {_endEffectorMotor.Set(0.5);}, [this] {_endEffectorMotor.Set(0);});
}

frc2::CommandPtr SubEndEffector::FeedDown() {
    return StartEnd([this] {_endEffectorMotor.Set(-0.5);}, [this] {_endEffectorMotor.Set(0);});
}

frc2::CommandPtr SubEndEffector::StopMotor() {
    return RunOnce([this] {_endEffectorMotor.Set(0);});
}

frc2::CommandPtr SubEndEffector::IntakeFromSource() {
    return FeedDown().Until([this] {return CheckLineBreak();});
}

frc2::CommandPtr SubEndEffector::IntakeFromGround() {
    return FeedUp().Until([this] {return LineBreakDownSignal();});
}


bool SubEndEffector::CheckLineBreak() {
    return _endEffectorLineBreak.Get();
}

frc2::Trigger SubEndEffector::CheckLineBreakTrigger() {
    return frc2::Trigger {[this] {return CheckLineBreak();}};
}

bool SubEndEffector::LineBreakDownSignal() {
    static bool PrevState = false;
    bool State = _endEffectorLineBreak.Get();
    if (PrevState == false && State == true) {
        return true;
    }
    else {
        PrevState = State;
        return false;
    }
}