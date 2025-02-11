// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubEndEffector.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "utilities/ICSpark.h"



SubEndEffector::SubEndEffector() {
    if(HasCoral = true) {SubEndEffector::GetInstance().KeepCoralInEndEffector();};
}

// This method will be called once per scheduler run
void SubEndEffector::Periodic() {
    frc::SmartDashboard::PutBoolean("EndEffector/Linebreak/1", SubEndEffector::GetInstance().CheckLineBreakHigher());
    frc::SmartDashboard::PutBoolean("EndEffector/Linebreak/2", SubEndEffector::GetInstance().CheckLineBreakLower());
    frc::SmartDashboard::PutNumber("EndEffector/Motor", _endEffectorMotor.Get());
}

frc2::CommandPtr SubEndEffector::FeedUp() {
    return StartEnd([this] {_endEffectorMotor.Set(0.8);}, [this] {_endEffectorMotor.Set(0);});
}

frc2::CommandPtr SubEndEffector::FeedUpSLOW() {
    return StartEnd([this] {_endEffectorMotor.Set(0.1);}, [this] {_endEffectorMotor.Set(0);});
}

frc2::CommandPtr SubEndEffector::FeedDown() {
    return StartEnd([this] {_endEffectorMotor.Set(-0.8);}, [this] {_endEffectorMotor.Set(0);});
}

frc2::CommandPtr SubEndEffector::FeedDownSLOW() {
    return StartEnd([this] {_endEffectorMotor.Set(-0.1);}, [this] {_endEffectorMotor.Set(0);});
}

frc2::CommandPtr SubEndEffector::Shoot() {
    return StartEnd([this] {_endEffectorMotor.Set(-0.8);}, [this] {_endEffectorMotor.Set(0);}).AndThen([this]{HasCoral = false;});
}

frc2::CommandPtr SubEndEffector::StopMotor() {
    return RunOnce([this] {
        _endEffectorMotor.Set(0);
        });
}



frc2::CommandPtr SubEndEffector::IntakeFromGround() {
    return FeedUp().Until([this] {return CheckLineBreakHigher();})
    .AndThen(FeedUpSLOW().Until([this] {return !CheckLineBreakLower();}))
    .AndThen(FeedDownSLOW().Until([this] {return CheckLineBreakLower();}))
    .AndThen([this] {HasCoral = true;});
}

frc2::CommandPtr SubEndEffector::ScoreCoral() {
    return Shoot();
}

frc2::CommandPtr SubEndEffector::ScoreCoralSLOW() {
    return FeedDownSLOW();
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

frc2::CommandPtr SubEndEffector::KeepCoralInEndEffector() {
return Run([this] {SubEndEffector::GetInstance().FeedUpSLOW();})
        .Until([this] {return !CheckLineBreakLower();});
}
