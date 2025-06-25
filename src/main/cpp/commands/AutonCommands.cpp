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

/* loops over a set of 6 constants for pathreef poses */
frc2::CommandPtr GenerateTeleopPath() {
    frc::Pose2d curpose = SubDrivebase::GetInstance().GetPose();
    frc::Pose2d endpose;

    /*
    REFACTOR TO PULL THIS LOGIC INTO GetDriveToScorePath;
    all logic about choosing paths should be in there
    */
    std::vector<frc::Pose2d> vec_poses;
    std::vector<std::pair<frc::Pose2d, frc::Pose2d>> posevec = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed
        ? SubVision::GetInstance().GetRedReefPoses()
        : SubVision::GetInstance().GetBlueReefPoses();
    std::vector<std::pair<frc::Pose2d, frc::Pose2d>>::iterator i;
    for (i = posevec.begin(); i != posevec.end(); i++) {
        vec_poses.push_back(i->first);
        vec_poses.push_back(i->second);
    }

    endpose = SubVision::GetInstance().GetReefPose(SubVision::Side::Left, 20); //curpose.Nearest(std::span<frc::Pose2d>(vec_poses));
    curpose = {curpose.X(), curpose.Y(), ICgeometry::PoseDirection(curpose, endpose)};
    /*rotation is the direction of the path*/
    // std::vector<frc::Pose2d> poses{curpose, endpose};
    // std::vector<pathplanner::Waypoint> waypoints =
    //     pathplanner::PathPlannerPath::waypointsFromPoses(poses);
    pathplanner::PathConstraints constraints(1.0_mps,          // max_speed
                                             1.0_mps_sq,       // max_accel
                                             360_deg_per_s,    // max_rotspeed
                                             360_deg_per_s_sq  // max_rotaccel
    );

    pathplanner::Pathfinding::ensureInitialized();
    pathplanner::Pathfinding::setStartPosition(curpose.Translation());


    Logger::Log("Auton/CurrentPose/x", curpose.Translation().X());
    Logger::Log("Auton/CurrentPose/y", curpose.Translation().Y());
    Logger::Log("Auton/CurrentPose/angle", curpose.Translation().Angle());

    return pathplanner::AutoBuilder::pathfindToPose(
        endpose,
        constraints,
        0.0_mps
    );
}

frc2::CommandPtr GetDriveToScorePath() {
  return frc2::cmd::DeferredProxy(
    [] { return GenerateTeleopPath(); }
  );
}

frc2::CommandPtr Score(int side) {
    return SubElevator::GetInstance().CmdSetL4()/*.OnlyIf([] { return !SubElevator::GetInstance().IsAtTarget(); })*/
        .AndThen(frc2::cmd::WaitUntil([]{ return SubElevator::GetInstance().IsAtTarget(); }))
        .AndThen(SubEndEffector::GetInstance().FeedDown().WithTimeout(0.5_s))
        .AndThen(SubElevator::GetInstance().CmdSetSource());
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
}  // namespace cmd