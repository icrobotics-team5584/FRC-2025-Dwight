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
  // Set up dev table
  _devTable.insert(0_m, 0);
  _devTable.insert(0.71_m, 0.002);
  _devTable.insert(1_m, 0.006);
  _devTable.insert(1.5_m, 0.02);
  _devTable.insert(2_m, 0.068);
  _devTable.insert(3_m, 0.230);

  // Robot pose estimater
  _leftPoseEstimater.SetMultiTagFallbackStrategy(photon::PoseStrategy::LOWEST_AMBIGUITY);

  // Sim set up
  _leftVisionSim.AddAprilTags(_tagLayout);
  _leftVisionSim.AddCamera(&_leftCamSim, _leftBotToCam);

  // Display tags on field
  for (auto target : _leftVisionSim.GetVisionTargets()) {
    SubDrivebase::GetInstance().DisplayPose(fmt::format("tag{}", target.fiducialId),
                                            target.GetPose().ToPose2d());
  }

  // Call this once just to get rid of the warnings that it is unused.
  // Its a photonlib bug.
  photon::VisionEstimation::EstimateCamPosePNP({}, {}, {}, {}, photon::TargetModel{1_m});
}

void SubVision::Periodic() {
  frc::SmartDashboard::PutNumber("Vision/LastReefTag", _lastReefTag.GetFiducialId());
  UpdateVision();
}

void SubVision::SimulationPeriodic() {
  _leftVisionSim.Update(SubDrivebase::GetInstance().GetSimPose());
}

void SubVision::UpdateVision() {
  std::optional<photon::PhotonTrackedTarget> bestTarget;
  double largestArea = 2;

  // Left camera
  std::vector<photon::PhotonPipelineResult> results = _leftCamera.GetAllUnreadResults();
  auto resultCount = results.size();
  if (resultCount > 0) {
    for (auto result : results) {
      _leftEstPose = _leftPoseEstimater.Update(result);
      frc::SmartDashboard::PutNumber("Vision/Left/target", result.GetBestTarget().fiducialId);

      for (const auto& target : result.targets) {
        if (!CheckReef(target)) {continue;}
        double targetArea = target.GetArea();
        if (targetArea > largestArea ) {
          bestTarget = target;
          largestArea = targetArea;
        }
      }
    }
  }
  // Right camera
  results = _rightCamera.GetAllUnreadResults();
  resultCount = results.size();
  if (resultCount > 0) {
    for (auto result : results) {
      _rightEstPose = _rightPoseEstimater.Update(result);
      frc::SmartDashboard::PutNumber("Vision/Right/target", result.GetBestTarget().fiducialId);

      for (const auto& target : result.targets) {
        if (CheckReef(target)) {continue;}
        double targetArea = target.GetArea();
        if (targetArea > largestArea) {
          bestTarget = target;
          largestArea = targetArea;
        }
      }
    }
  }

  if (bestTarget) {
    _lastReefTag = *bestTarget;
  }
}

std::map<SubVision::Side, std::optional<photon::EstimatedRobotPose>> SubVision::GetPose() {
  return {{Left, _leftEstPose}, {Right, _rightEstPose}};
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
frc::Pose2d SubVision::GetReefPose(Side side = Left, int pose = -1) {
  int reefTagID = (pose == -1)? _lastReefTag.GetFiducialId() : pose;
  frc::Pose2d targPose;
  if (side == Left) {
    targPose = {tagToReefPositions[reefTagID].leftX, tagToReefPositions[reefTagID].leftY,
                tagToReefPositions[reefTagID].angle};
  } else {
    targPose = {tagToReefPositions[reefTagID].rightX, tagToReefPositions[reefTagID].rightY,
                tagToReefPositions[reefTagID].angle};
  }
  return targPose;
}

units::degree_t SubVision::GetReefAlignAngle(Side side = Left) {
  int reefTagID = _lastReefTag.GetFiducialId();
  if (side == Left) {
    return tagToReefPositions[reefTagID].leftScoreAngle;
  } else {
    return tagToReefPositions[reefTagID].rightScoreAngle;
  }
  return 0_deg;
}

units::degree_t SubVision::GetLastReefTagAngle() {
  return _lastReefTag.GetYaw() * 1_deg;
}

double SubVision::GetLastReefTagArea() {
  return _lastReefTag.GetArea();
}

bool SubVision::CheckReef(const photon::PhotonTrackedTarget& reef) {
  const auto& myReef =
      (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) ? redReef : blueReef;
  if (std::find(myReef.begin(), myReef.end(), reef.GetFiducialId()) != myReef.end()) {
    return false;
  }
  units::degree_t errorAngle = SubDrivebase::GetInstance().GetPose().Rotation().Degrees() -
                               GetReefPose(Left,reef.GetFiducialId()).Rotation().Degrees() - 180_deg;
  if ((errorAngle > 30_deg || errorAngle < -30_deg)) {
    return false;
  }
  return true;
}

double SubVision::GetDev(photon::EstimatedRobotPose pose) {
  units::meter_t distance = 0_m;
  if (pose.targetsUsed.size() == 0) {
    return 0;
  }
  for (auto target : pose.targetsUsed) {
    distance += target.GetBestCameraToTarget().Translation().Norm();
  }
  distance /= pose.targetsUsed.size();
  return _devTable[distance];
}