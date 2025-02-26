// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubEndEffector.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "utilities/ICSpark.h"
#include "utilities/BotVars.h"



SubEndEffector::SubEndEffector() {
    rev::spark::SparkBaseConfig config;
    config.Inverted(BotVars::Choose(true, false));
    _endEffectorMotor.AdjustConfig(config);
    frc::SmartDashboard::PutData("EndEffector/motorData", &_endEffectorMotor);
}

// This method will be called once per scheduler run
void SubEndEffector::Periodic() {
    frc::SmartDashboard::PutBoolean("EndEffector/LinebreakHigher", SubEndEffector::GetInstance().CheckLineBreakHigher());
    frc::SmartDashboard::PutBoolean("EndEffector/LinebreakLower", SubEndEffector::GetInstance().CheckLineBreakLower());
    frc::SmartDashboard::PutNumber("EndEffector/endEffectorMotor", _endEffectorMotor.Get());
    frc::SmartDashboard::PutNumber("EndEffector/MotorCurrent", _endEffectorMotor.GetOutputCurrent());
}

frc2::CommandPtr SubEndEffector::FeedUp() {
    return StartEnd([this] {_endEffectorMotor.Set(0.3);}, [this] {_endEffectorMotor.Set(0);});
}

frc2::CommandPtr SubEndEffector::FeedUpSLOW() {
    return StartEnd([this] {_endEffectorMotor.Set(0.05);}, [this] {_endEffectorMotor.Set(0);});
}

frc2::CommandPtr SubEndEffector::FeedDown() {
    return StartEnd([this] {_endEffectorMotor.Set(-0.3);}, [this] {_endEffectorMotor.Set(0);});
}

frc2::CommandPtr SubEndEffector::FeedDownSLOW() {
    return StartEnd([this] {_endEffectorMotor.Set(-0.05);}, [this] {_endEffectorMotor.Set(0);});
}

frc2::CommandPtr SubEndEffector::Shoot() {
    return StartEnd([this] {_endEffectorMotor.Set(-0.3);}, [this] {_endEffectorMotor.Set(0);});
}

frc2::CommandPtr SubEndEffector::StopMotor() {
    return RunOnce([this] {
        _endEffectorMotor.Set(0);
        });
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
  return Run([this] {
    if (CheckLineBreakHigher() && !CheckLineBreakLower()) {
        frc::SmartDashboard::PutString("EndEffector/Coral Position", "Too High");
        _endEffectorMotor.Set(-0.3);
    } else if (CheckLineBreakLower() && !CheckLineBreakHigher()) {
        frc::SmartDashboard::PutString("EndEffector/Coral Position", "Too Low");
        _endEffectorMotor.Set(0.3);
    } else {
        frc::SmartDashboard::PutString("EndEffector/Coral Position", "Just Right");
        _endEffectorMotor.Set(0);
    }
  });
}

