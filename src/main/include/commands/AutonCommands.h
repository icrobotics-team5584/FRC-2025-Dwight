// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <units/angular_velocity.h>
#include <units/length.h>

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include "subsystems/SubVision.h"

enum ReefSide {
    FrontRight = 0,
    FrontMiddle = 1,
    FrontLeft = 2,
    BackLeft = 3,
    BackMiddle = 4,
    BackRight = 5,
};

struct PathPlannerReefPosition {
    units::meter_t x;
    units::meter_t y;
    units::degree_t angle;
};

namespace cmd {
    frc2::CommandPtr AutonSubSystemsZeroSequence();
    
    std::shared_ptr<pathplanner::PathPlannerPath> GenerateTeleopPath();
    frc2::CommandPtr GetTeleopPathCommand(std::string pathName);

    frc2::CommandPtr ScoreWithVision(SubVision::Side side);
    
    frc2::CommandPtr Score(int side);
    
    frc2::CommandPtr AutonBeginSourceIntake();
    frc2::CommandPtr AutonEndSourceIntake();
}