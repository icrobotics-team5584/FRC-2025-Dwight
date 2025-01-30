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
  // Pitch table set up
  _devTable.insert(0_m, 0.03);
  _devTable.insert(0.5_m, 0.05);
  _devTable.insert(1_m, 0.1);
  _devTable.insert(2_m, 0.2);
  _devTable.insert(3_m, 0.3);
  _devTable.insert(4_m, 0.4);
  _devTable.insert(5_m, 0.5);

  // Robot pose estimater
  _robotPoseEstimater.SetMultiTagFallbackStrategy(photon::PoseStrategy::LOWEST_AMBIGUITY);


  // Sim set up
  _visionSim.AddAprilTags(_tagLayout);
  _visionSim.AddCamera(&_cameraSim, _botToCam);

  // Display tags on field
  for (auto target : _visionSim.GetVisionTargets()) {
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
    auto distance = result.GetBestTarget().GetBestCameraToTarget().Translation().Distance(frc::Translation3d(0_m,0_m,0_m));
    lastDev = _devTable[distance];
    frc::SmartDashboard::PutNumber("Vision/distance to tag", distance.value());
    frc::SmartDashboard::PutNumber("Vision/target", result.GetBestTarget().fiducialId);
    frc::SmartDashboard::PutNumber("Vision/dev", lastDev);
  }
}

void SubVision::UpdateLatestTags(std::vector<photon::PhotonPipelineResult> results) {
  if (results.size() == 0) {return;}
  for (auto result : results) {
    if (result.HasTargets()) {
      _latestTarget = result.GetBestTarget();
      frc::SmartDashboard::PutNumber("Vision/Target", _latestTarget.GetFiducialId());
      if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) {
        if (std::find(std::begin(redReef), std::end(redReef), _latestTarget.GetFiducialId()) !=
            std::end(redReef)) {
          _lastReefTag = _latestTarget;
        }
      } else {
        if (std::find(std::begin(blueReef), std::end(blueReef), _latestTarget.GetFiducialId()) !=
            std::end(blueReef)) {
          _lastReefTag = _latestTarget;
        }
      }
    }
  }
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

double SubVision::GetLastDev() {
  return lastDev;
}
