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

namespace cmd {
    frc2::CommandPtr AutonSubSystemsZeroSequence();
    
    frc2::CommandPtr GenerateTeleopPath(frc::Pose2d startpose, frc::Pose2d endpose);

    frc2::CommandPtr ScoreWithVision(SubVision::Side side);

    frc2::CommandPtr ScoreWithTeleop(SubVision::Side side, int pose);
    
    frc2::CommandPtr Score(int side);
    
    frc2::CommandPtr AutonBeginSourceIntake();
    frc2::CommandPtr AutonEndSourceIntake();
}