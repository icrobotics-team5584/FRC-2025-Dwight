// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubEndEffector.h"
#include <frc/smartdashboard/SmartDashboard.h>



SubEndEffector::SubEndEffector() = default;

// This method will be called once per scheduler run
void SubEndEffector::Periodic() {
    frc::SmartDashboard::PutBoolean("EndEffector/Linebreak/1", SubEndEffector::GetInstance().CheckLineBreakHigher());
    frc::SmartDashboard::PutBoolean("EndEffector/Linebreak/2", SubEndEffector::GetInstance().CheckLineBreakHigher());
}

frc2::CommandPtr SubEndEffector::FeedUp() {
    return StartEnd([this] {_endEffectorMotor.Set(0.3);}, [this] {_endEffectorMotor.Set(0);});
}

frc2::CommandPtr SubEndEffector::FeedUpSLOWER() {
    return StartEnd([this] {_endEffectorMotor.Set(0.05);}, [this] {_endEffectorMotor.Set(0);});
}

frc2::CommandPtr SubEndEffector::FeedDown() {
    return StartEnd([this] {_endEffectorMotor.Set(-0.3);}, [this] {_endEffectorMotor.Set(0);});
}

frc2::CommandPtr SubEndEffector::FeedDownSLOWER() {
    return StartEnd([this] {_endEffectorMotor.Set(-0.05);}, [this] {_endEffectorMotor.Set(0);});
}

frc2::CommandPtr SubEndEffector::StopMotor() {
    return RunOnce([this] {_endEffectorMotor.Set(0);});
}

frc2::CommandPtr SubEndEffector::IntakeFromSource() {
    return FeedDown().Until([this] {return CheckLineBreakHigher();}).AndThen(FeedDownSLOWER().Until([this] {return CheckLineBreakLower();}));
}

frc2::CommandPtr SubEndEffector::IntakeFromGround() {
    return FeedUp().Until([this] {return CheckLineBreakLower();})
    .AndThen(FeedUpSLOWER().Until([this] {return CheckLineBreakHigher();}))
    .AndThen(FeedUpSLOWER().Until([this] {return !CheckLineBreakLower();}))
    .AndThen(FeedDownSLOWER().Until([this] {return CheckLineBreakLower();}));
}


bool SubEndEffector::CheckLineBreakHigher() {
    return _endEffectorLineBreakHigher.Get();
}

bool SubEndEffector::CheckLineBreakLower() {
    return _endEffectorLineBreakLower.Get();
}

frc2::Trigger SubEndEffector::CheckLineBreakTriggerHigher() {
    return frc2::Trigger {[this] {return this->CheckLineBreakHigher();}};
}

frc2::Trigger SubEndEffector::CheckLineBreakTriggerLower() {
    return frc2::Trigger {[this] {return this->CheckLineBreakLower();}};
}
