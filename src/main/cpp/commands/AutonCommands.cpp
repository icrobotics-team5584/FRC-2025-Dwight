// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <RobotContainer.h>

#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

#include "subsystems/SubEndEffector.h"
#include "subsystems/SubDrivebase.h"
#include "subsystems/SubElevator.h"
#include "subsystems/SubFunnel.h"
#include "subsystems/SubClimber.h"

#include "commands/VisionCommand.h"
#include "commands/GamePieceCommands.h"
#include "commands/AutonCommands.h"

namespace cmd {
frc2::CommandPtr AutonSubSystemsZeroSequence() {
  return frc2::cmd::Parallel(SubElevator::GetInstance().ElevatorAutoReset(),
                             SubClimber::GetInstance().ClimberAutoReset());
}

/* loops over a set of 6 constants for pathreef poses */
std::shared_ptr<pathplanner::PathPlannerPath> GenerateTeleopPath() {
    // TODO: MOVE FIND CLOSEST POSE TO ITS OWN FUNCTION; THINK ABT MOVING INTO DIFFERENT FILE

    frc::Pose2d curpose = SubDrivebase::GetInstance().GetPose();
    frc::Pose2d endpose;
    double shortestDistance = 100;
    double curDistance;
    int closestindex;
    // TODO: FIX THE HEADER STDMAP ISSUES
    /* this is workaround for C/C++ 289; was hoping to define this in <AutonCommands.h> but kept
     * getting C/C++ 289 */
    PathPlannerReefPosition PathReefPoses[6]{
        {3.62_m, 2.91_m, -30_deg},   // FR
        {3.22_m, 4.025_m, -90_deg},  // FM
        {3.84_m, 5.12_m, 210_deg},   // FL
        {5.11_m, 5.14_m, 150_deg},   // BL
        {5.76_m, 4.04_m, 90_deg},    // BM
        {5.15_m, 2.94_m, 30_deg}     // BR
    };

    for (int i = 0; i < 6; i++) {
      frc::Pose2d pathpose = frc::Pose2d(PathReefPoses[i].x, PathReefPoses[i].y,
                                         frc::Rotation2d(PathReefPoses[i].angle));
      curDistance = curpose.Translation().Distance(pathpose.Translation()).value();
      if (curDistance < shortestDistance) {
        shortestDistance = curDistance;
        closestindex = i;
        Logger::Log("Auton/DistanceToIndex[" + std::to_string(closestindex) + "]", curDistance);
      }
      Logger::Log("Auton/EndPoseIndex", static_cast<double>(closestindex));
    }
    endpose = frc::Pose2d(PathReefPoses[closestindex].x, PathReefPoses[closestindex].y,
                          frc::Rotation2d(PathReefPoses[closestindex].angle));
    // rotation is the direction of the path
    std::vector<frc::Pose2d> poses{curpose, endpose};
    std::vector<pathplanner::Waypoint> waypoints =
        pathplanner::PathPlannerPath::waypointsFromPoses(poses);
    pathplanner::PathConstraints constraints(1.0_mps,          // max_speed
                                             1.0_mps_sq,       // max_accel
                                             360_deg_per_s,    // max_rotspeed
                                             360_deg_per_s_sq  // max_rotaccel
    );
    pathplanner::PathPlannerPath path(
        waypoints, constraints, std::nullopt,
        pathplanner::GoalEndState(0.0_mps,
        endpose.Rotation() /*.RotateBy(frc::Rotation2d(180_deg))*/));

    Logger::Log("Auton/CurrentPose/x", curpose.Translation().X());
    Logger::Log("Auton/CurrentPose/y", curpose.Translation().Y());
    Logger::Log("Auton/CurrentPose/angle", curpose.Translation().Angle());

    return std::make_shared<pathplanner::PathPlannerPath>(path);
}

frc2::CommandPtr GetTeleopPath() {
  return frc2::cmd::DeferredProxy(
    [] { return pathplanner::AutoBuilder::followPath(GenerateTeleopPath()); }
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