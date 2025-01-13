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
    return FeedDown().Until([this] {return CheckLineBreakLower();});
}

frc2::CommandPtr SubEndEffector::IntakeFromGround() {
    return FeedUp().Until([this] {return CheckLineBreakHigher();});
}


bool SubEndEffector::CheckLineBreakHigher() {
    return _endEffectorLineBreakHigher.Get();
}

bool SubEndEffector::CheckLineBreakLower() {
    return _endEffectorLineBreakLower.Get();
}

frc2::Trigger SubEndEffector::CheckLineBreakTriggerHigher() {
    return frc2::Trigger {[this] {return CheckLineBreakHigher();}};
}

frc2::Trigger SubEndEffector::CheckLineBreakTriggerLower() {
    return frc2::Trigger {[this] {return CheckLineBreakLower();}};
}
