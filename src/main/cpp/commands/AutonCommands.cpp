// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <RobotContainer.h>

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/pathfinding/Pathfinding.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include "subsystems/SubEndEffector.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubElevator.h"
#include "subsystems/SubVision.h"
#include "subsystems/SubFunnel.h"
#include "subsystems/SubClimber.h"

#include "commands/VisionCommand.h"
#include "commands/GamePieceCommands.h"
#include "commands/AutonCommands.h"

#include "utilities/ICgeometry.h"

namespace cmd {
frc2::CommandPtr AutonSubSystemsZeroSequence() {
  return frc2::cmd::Parallel(SubElevator::GetInstance().ElevatorAutoReset(),
                             SubClimber::GetInstance().ClimberAutoReset());
}

frc2::CommandPtr GenerateTeleopPath(frc::Pose2d startpose, frc::Pose2d endpose) {
    pathplanner::PathConstraints constraints(1.0_mps,          // max_speed
                                             1.0_mps_sq,       // max_accel
                                             360_deg_per_s,    // max_rotspeed
                                             360_deg_per_s_sq  // max_rotaccel
    );

    pathplanner::Pathfinding::ensureInitialized();
    pathplanner::Pathfinding::setStartPosition(startpose.Translation());


    Logger::Log("Auton/CurrentPose/x", startpose.Translation().X());
    Logger::Log("Auton/CurrentPose/y", startpose.Translation().Y());
    Logger::Log("Auton/CurrentPose/angle", startpose.Translation().Angle());

    return pathplanner::AutoBuilder::pathfindToPose(
        endpose,
        constraints,
        0.0_mps
    );
}

frc2::CommandPtr Score(int side) {
    return SubElevator::GetInstance().CmdSetL4()/*.OnlyIf([] { return !SubElevator::GetInstance().IsAtTarget(); })*/
        .AndThen(frc2::cmd::WaitUntil([]{ return SubElevator::GetInstance().IsAtTarget(); }))
        .AndThen(SubEndEffector::GetInstance().FeedDown().WithTimeout(0.5_s))
        .AndThen(SubElevator::GetInstance().CmdSetSource());
}

frc2::CommandPtr ScoreWithTeleop(SubVision::Side side, int pose) {
  frc::Pose2d curpose = SubDrivebase::GetInstance().GetPose();
  frc::Pose2d endpose = SubVision::GetInstance().GetReefPose(side, pose);
  return GenerateTeleopPath(curpose, endpose)
    .AndThen(Score(side));
}

// vision
frc2::CommandPtr ScoreWithVision(SubVision::Side side) {
  static frc::Timer timer;

  return SubElevator::GetInstance()
      .CmdSetL4()  //.OnlyIf([] { return !SubElevator::GetInstance().IsAtTarget(); })
      .AndThen(frc2::cmd::WaitUntil([] { return SubElevator::GetInstance().IsAtTarget(); }))
      .AndThen([] { timer.Restart(); })
      .AndThen(cmd::AlignAndShoot(side).Until([] {
        bool has_coral = SubEndEffector::GetInstance().CheckLineBreakHigher();
        if (has_coral == false) {
          Logger::Log("EndEffector/ScoreWithVision/has coral", false);
          timer.Start();
        } else {
          Logger::Log("EndEffector/ScoreWithVision/has coral", true);
          timer.Restart();
        }
        Logger::Log("EndEffector/ScoreWithVision/time elapsed without coral", timer.Get());
        return timer.HasElapsed(0.5_s);
      }))
      .AndThen(SubElevator::GetInstance().CmdSetSource())
      .AndThen(SubEndEffector::GetInstance().RemoveAlgae().Until(
          [] { return SubElevator::GetInstance().IsAtTarget(); }));
}

frc2::CommandPtr AutonBeginSourceIntake() {
  return SubElevator::GetInstance()
      .CmdSetSource()
      .AndThen(frc2::cmd::WaitUntil([] { return SubElevator::GetInstance().IsAtTarget(); }))
      .AndThen(SubFunnel::GetInstance().FeedDownFunnel())
      .AlongWith(SubEndEffector::GetInstance().FeedDown())
      .Until([] { return SubEndEffector::GetInstance().CheckLineBreakHigher(); });
}

frc2::CommandPtr AutonEndSourceIntake() {
  return SubEndEffector::GetInstance()
      .FeedDownSLOW()
      .AlongWith(SubFunnel::GetInstance().FeedDownFunnelSLOW())
      .Until([] { return SubEndEffector::GetInstance().CheckLineBreakLower(); });
}
}  // namespace cmd