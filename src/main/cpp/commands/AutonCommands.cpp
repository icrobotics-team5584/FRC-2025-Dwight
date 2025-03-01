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
#include "subsystems/SubFunnel.h"
#include "subsystems/SubClimber.h"

#include "commands/VisionCommand.h"
#include "commands/GamePieceCommands.h"

namespace cmd {
    frc2::CommandPtr AutonSubSystemsZeroSequence() {
        return SubElevator::GetInstance().ElevatorAutoReset()
            .AndThen(SubElevator::GetInstance().CmdSetL1())
            .AndThen(frc2::cmd::WaitUntil([]{ return SubElevator::GetInstance().IsAtTarget(); }))
            .AndThen(SubClimber::GetInstance().ClimberAutoReset())
            .AndThen(SubElevator::GetInstance().CmdSetSource());
    }

    
    //  no vision
    frc2::CommandPtr Score(int side) {
        return SubElevator::GetInstance().CmdSetL4()//.OnlyIf([] { return !SubElevator::GetInstance().IsAtTarget(); })
            .AndThen(frc2::cmd::WaitUntil([]{ return SubElevator::GetInstance().IsAtTarget(); }))
            .AndThen(SubEndEffector::GetInstance().FeedDown().WithTimeout(0.5_s))
            .AndThen(SubElevator::GetInstance().CmdSetSource());
    }

    frc2::CommandPtr ScoreWithVision(SubVision::Side side) {
        static frc::Timer timer;

        return SubElevator::GetInstance().CmdSetL4()//.OnlyIf([] { return !SubElevator::GetInstance().IsAtTarget(); })
            .AndThen(frc2::cmd::WaitUntil([]{ return SubElevator::GetInstance().IsAtTarget(); }))
            .AndThen([]{timer.Restart();})
            .AndThen(cmd::AlignAndShoot(side).Until([]{ 
                bool has_coral = SubEndEffector::GetInstance().CheckLineBreakLower() && SubEndEffector::GetInstance().CheckLineBreakHigher();
                if ( has_coral == false) {
                    Logger::Log("EndEffector/ScoreWithVision/has coral", false);
                    timer.Start();
                } else {
                    Logger::Log("EndEffector/ScoreWithVision/has coral", true);
                    timer.Restart();
                }
                Logger::Log("EndEffector/ScoreWithVision/time elapsed without coral", timer.Get());
                return timer.HasElapsed(0.5_s); }))
            .AndThen(SubElevator::GetInstance().CmdSetSource());
    }
}