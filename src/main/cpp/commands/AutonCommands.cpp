// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <RobotContainer.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include "subsystems/SubEndEffector.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubElevator.h"
#include "subsystems/SubFunnel.h"
#include "subsystems/SubClimber.h"

#include "commands/VisionCommand.h"
#include "commands/GamePieceCommands.h"

#include "utilities/LEDHelper.h"

namespace cmd {
    frc2::CommandPtr AutonSubSystemsZeroSequence() {
        return frc2::cmd::Parallel(
            SubElevator::GetInstance().ElevatorAutoReset(),
            SubClimber::GetInstance().ClimberAutoReset()
        );
    }

    
    //  no vision
    frc2::CommandPtr Score(int side) {
        return SubElevator::GetInstance().CmdSetL4()//.OnlyIf([] { return !SubElevator::GetInstance().IsAtTarget(); })
            .AndThen(frc2::cmd::WaitUntil([]{ return SubElevator::GetInstance().IsAtTarget(); }))
            .AndThen(SubEndEffector::GetInstance().FeedDown().WithTimeout(0.5_s))
            .AndThen(SubElevator::GetInstance().CmdSetSource());
    }

    frc2::CommandPtr AutonBeginSourceIntake() {
        return IntakeFromSource();
    }
    
    frc2::CommandPtr AutonEndSourceIntake() {
        return SubFunnel::GetInstance().FeedDownFunnelSLOW()
            .AlongWith(SubEndEffector::GetInstance().FeedDown())
            .Until([] { return SubEndEffector::GetInstance().CheckLineBreakLower(); })
            .AndThen(SubEndEffector::GetInstance().StopMotor());
    }


    // vision
    frc2::CommandPtr ScoreWithVision(SubVision::Side side) {
        static frc::Timer timer;

        return SubElevator::GetInstance().CmdSetL4()//.OnlyIf([] { return !SubElevator::GetInstance().IsAtTarget(); })
            .AndThen(frc2::cmd::WaitUntil([]{ return SubElevator::GetInstance().IsAtTarget(); }))
            .AndThen([]{timer.Restart();})
            .AndThen(cmd::AlignAndShoot(side).Until([]{ 
                bool has_coral = SubEndEffector::GetInstance().CheckLineBreakHigher();
                if ( has_coral == false) {
                    Logger::Log("EndEffector/ScoreWithVision/has coral", false);
                    timer.Start();
                } else {
                    Logger::Log("EndEffector/ScoreWithVision/has coral", true);
                    timer.Restart();
                }
                Logger::Log("EndEffector/ScoreWithVision/time elapsed without coral", timer.Get());
                return timer.HasElapsed(0.5_s); }));
    }

    frc2::CommandPtr ScoreWith3DVision(SubVision::Side side) {
        static frc::Pose2d targetAwayPose;
        static frc::Pose2d targetTagPose;
        static frc::Pose2d awayPose;
        static units::meter_t initialDistance = 0_m;

        //Prepare all positions
        return frc2::cmd::RunOnce([side] {
            int tagId = SubVision::GetInstance().GetClosestTag(SubDrivebase::GetInstance().GetPose());
            targetTagPose = SubVision::GetInstance().GetReefPose(tagId, side);
            targetAwayPose = SubVision::GetInstance().CalculateRelativePose(targetTagPose,0_m,-0.4_m);
            initialDistance = SubDrivebase::GetInstance().TranslationPosDistance(targetAwayPose)*1_m;
            frc::SmartDashboard::PutNumber("Closest tag", tagId);
            SubDrivebase::GetInstance().DisplayPose("Vision 3d align pose", targetAwayPose);
            SubDrivebase::GetInstance().DisplayPose("Vision 3d align target pose", targetTagPose);
        })
        // // Drive to roughly half a meter away from pose
        // .AndThen(
        //     SubDrivebase::GetInstance().DriveToPose([](){return targetAwayPose;}, 3)
        //     .DeadlineFor(LEDHelper::GetInstance().SetFollowProgress([] {return SubDrivebase::GetInstance().TranslationPosError(targetAwayPose, initialDistance);}, frc::Color::kAliceBlue)))
        // // Bring elevator up
        .AndThen(
            cmd::SetElevatorPosition([](){return SubElevator::GetInstance().GetPresetHeight();},true)
            .AndThen(
            [] {initialDistance = SubDrivebase::GetInstance().TranslationPosDistance(targetTagPose)*1_m;}) /*LED stuff*/
        // Drive close to reef
        .AndThen(
            SubDrivebase::GetInstance().DriveToPose([](){return targetTagPose;}, 0.5).WithDeadline(frc2::cmd::Wait(2.5_s))
            .DeadlineFor(LEDHelper::GetInstance().SetFollowProgress([] {return SubDrivebase::GetInstance().TranslationPosError(targetTagPose, initialDistance);}, frc::Color::kGreen)))
        //Score coral
        .AndThen(SubEndEffector::GetInstance().ScoreCoral().WithTimeout(0.4_s)));
        // Lower elevator
        //.AndThen(SubElevator::GetInstance().CmdSetSource()));
    }


    frc2::CommandPtr ScoreWithPrescription(SubVision::Side side) {
        static frc::Timer timer;

        return SubElevator::GetInstance().CmdSetL4()//.OnlyIf([] { return !SubElevator::GetInstance().IsAtTarget(); })
            .AndThen(frc2::cmd::WaitUntil([]{ return SubElevator::GetInstance().IsAtTarget(); }))
            .AndThen([]{timer.Restart();})
            .AndThen(cmd::HopeAndShoot(side).Until([]{ 
                bool has_coral = SubEndEffector::GetInstance().CheckLineBreakHigher();
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