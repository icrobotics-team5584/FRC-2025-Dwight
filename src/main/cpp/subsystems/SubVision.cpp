// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubVision.h"
#include "subsystems/SubDrivebase.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <photon/estimation/CameraTargetRelation.h>

SubVision::SubVision() {
  _robotPoseEstimater.SetMultiTagFallbackStrategy(photon::PoseStrategy::LOWEST_AMBIGUITY);

  _visionSim.AddAprilTags(_tagLayout);  // Configure vision sim
  _visionSim.AddCamera(&_cameraSim, _botToCam);

  for (auto target : _visionSim.GetVisionTargets()) {  // Add AprilTags to field visualization
    SubDrivebase::GetInstance().DisplayPose(
        fmt::format("tag{}", target.fiducialId),
        target.GetPose().ToPose2d());
  }

  // Call this once just to get rid of the warnings that it is unused.
  // Its a photonlib bug.
  photon::VisionEstimation::EstimateCamPosePNP({}, {}, {}, {}, photon::TargetModel{1_m});
}

void SubVision::Periodic() {
  frc::SmartDashboard::PutNumber("Vision/LastReefTag", _lastReefTag.GetFiducialId());
  auto results = _camera.GetAllUnreadResults();
  UpdatePoseEstimator(results);
  UpdateLatestTags(results);
}

void SubVision::SimulationPeriodic() {
  _visionSim.Update(SubDrivebase::GetInstance().GetSimPose());
}

void SubVision::UpdatePoseEstimator(std::vector<photon::PhotonPipelineResult> results) {
  auto resultCount = results.size();
  frc::SmartDashboard::PutNumber("Vision/Reuslt count", resultCount);
  if (resultCount == 0) {
    return;  // Return if no result, prevent null error later
  }
  for (auto result : results) {
    _pose = _robotPoseEstimater.Update(result);  // Get estimate pose of robot by vision
  }
}
void SubVision::UpdateLatestTags(std::vector<photon::PhotonPipelineResult> results) {
  std::string currentlyVisibleTagIDs;
  std::optional<photon::PhotonTrackedTarget> largestTarget;
  double largestArea = 0.0;
  const auto& myReef =
      (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) ? redReef : blueReef;

  for (const auto& result : results) {
    for (const auto& target : result.targets) {
      currentlyVisibleTagIDs += std::to_string(target.GetFiducialId()) + " ";
      double targetArea = target.GetArea();

      if (std::find(myReef.begin(), myReef.end(), target.GetFiducialId()) != myReef.end()) {
        if (targetArea > largestArea) {
          largestTarget = target;
          largestArea = targetArea;
        }
      }
    }
  }

  if (largestTarget) {
    _lastReefTag = *largestTarget;
    _lastReefArea = largestArea;
  }

  frc::SmartDashboard::PutString("Vision/Currently visible tags", currentlyVisibleTagIDs);
}

std::optional<photon::EstimatedRobotPose> SubVision::GetPose() {
  return _pose;
}

frc::Pose2d SubVision::GetSourcePose(int tagId) {
  return tagToSourcePose[tagId];
}

/**
 * @brief Get the pose of the reef from the apriltag.
 *
 * @param side The side of the reef you want to get the pose of. 1 for the left side, 2 for the
 * right side.
 *
 * @returns The pose of the reef in the field coordinate system.
 */
frc::Pose2d SubVision::GetReefPose(int side = 1) {
  int reefTagID = _lastReefTag.GetFiducialId();
  frc::Pose2d targPose;
  if (side == 1) {
    targPose = {tagToReefPositions[reefTagID].leftX, tagToReefPositions[reefTagID].leftY,
                tagToReefPositions[reefTagID].angle};
  } else {
    targPose = {tagToReefPositions[reefTagID].rightX, tagToReefPositions[reefTagID].rightY,
                tagToReefPositions[reefTagID].angle};
  }
  return targPose;
}

units::degree_t SubVision::GetReefAlignAngle(int side = 1) {
  int reefTagID = _lastReefTag.GetFiducialId();
  if (side == 1) {
    return tagToReefPositions[reefTagID].leftScoreAngle;
  } else {
    return tagToReefPositions[reefTagID].rightScoreAngle;
  }
  return 0_deg;
}

units::degree_t SubVision::GetReefTagAngle(){
  return _lastReefTag.GetYaw()*1_deg;
}

double SubVision::GetReefArea() {
  return _lastReefArea;
}

bool SubVision::CheckValid(std::optional<photon::EstimatedRobotPose> pose) {
  const double minArea = 20;
  const double maxArea = 80;
  const double minAmbiguity = 0.2;
  const double maxAmbiguity = 0.8;
  const int desiredTargetLimit = 3;

  auto data = pose.value();

  frc::SmartDashboard::PutNumber("Vision/Target detected",
                                 data.targetsUsed.size());  // Display number of target detected

  // Check number of target
  if (data.targetsUsed.size() > desiredTargetLimit) {
    return false;
  }

  // Get average area and ambiguity
  double area = 0.00;
  double ambiguity = 0.00;
  for (auto target : data.targetsUsed) {
    area += target.GetArea();
    ambiguity += target.GetPoseAmbiguity();
  }
  area /= data.targetsUsed.size();
  ambiguity /= data.targetsUsed.size();
  frc::SmartDashboard::PutNumber("Vision/Target avg area", area);
  frc::SmartDashboard::PutNumber("Vision/Target avg ambiguity", ambiguity);

  // Check area
  if (area < minArea || area > maxArea) {
    return false;
  }

  // Check ambiguity
  if ((ambiguity < minAmbiguity || ambiguity > maxAmbiguity) && !frc::RobotBase::IsSimulation()) {
    return false;
  }

  return true;
}
