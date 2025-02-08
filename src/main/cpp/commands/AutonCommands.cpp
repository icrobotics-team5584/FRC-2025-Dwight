// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <RobotContainer.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "subsystems/SubEndEffector.h"
#include "subsystems/SubElevator.h"

#include "commands/VisionCommand.h"

namespace cmd {
    frc2::CommandPtr ScoreWithVision(int side) {
        return frc2::cmd::Wait(1.0_s) //.AndThen(cmd::YAutonAlignWithTarget(1)) // vision is smucked (once fixed replace )
            .AndThen(SubElevator::GetInstance().CmdSetL2())
            .AndThen(cmd::AlignAndShoot(side, SubElevator::_L4_HEIGHT))
            // .AndThen(cmd::ForceAlignWithTarget(side).WithTimeout(6_s))
            // .AndThen(SubElevator::GetInstance().CmdSetL4().WithName("CmdSetL4")
            // .AndThen(frc2::cmd::WaitUntil([]{ return SubElevator::GetInstance().IsAtTarget() == true; })))
            // .AndThen(frc2::cmd::Wait(0.2_s))
            // .AndThen(SubEndEffector::GetInstance().ScoreCoral().WithName("ScoreCoral").WithTimeout(2_s))
            .AndThen(SubElevator::GetInstance().CmdSetSource().WithName("CmdSetSource"))
            .AndThen(frc2::cmd::Wait(0.2_s));
    }

    frc2::CommandPtr IntakeSourceWithVision() {
        return frc2::cmd::Wait(2.0_s)
            .AndThen(SubElevator::GetInstance().CmdSetSource().WithName("CmdSetSource"))
            .Until([]{return SubElevator::GetInstance().IsAtTarget() == true;})
            .AndThen(SubEndEffector::GetInstance().IntakeFromSource().WithName("IntakeFromSource").WithTimeout(1_s));
    }
}