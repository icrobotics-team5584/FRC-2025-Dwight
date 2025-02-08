// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <pathplanner/lib/auto/NamedCommands.h>


#include "subsystems/SubEndEffector.h"
#include "subsystems/SubElevator.h"

namespace cmd {
    frc2::CommandPtr ScoreWithVision(int side);
    frc2::CommandPtr IntakeSourceWithVision();
}