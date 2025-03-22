// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

namespace cmd {
    frc2::CommandPtr AutonSubSystemsZeroSequence();
    
    std::shared_ptr<pathplanner::PathPlannerPath> GenerateTeleopPath();
    frc2::CommandPtr GetTeleopPathCommand(std::string pathName);

    frc2::CommandPtr ScoreWithVision(SubVision::Side side);
    
    frc2::CommandPtr Score(int side);
    
    frc2::CommandPtr AutonBeginSourceIntake();
    frc2::CommandPtr AutonEndSourceIntake();
}