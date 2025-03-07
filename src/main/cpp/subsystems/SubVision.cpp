// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubVision.h"
#include "subsystems/SubDrivebase.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <photon/estimation/CameraTargetRelation.h>
#include <frc/MathUtil.h>

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
  _visionSim.AddAprilTags(_tagLayout);
  _visionSim.AddCamera(&_leftCamSim, _leftBotToCam);
  _visionSim.AddCamera(&_rightCamSim, _rightBotToCam);

  // Display tags on field
  for (auto target : _visionSim.GetVisionTargets()) {
    SubDrivebase::GetInstance().DisplayPose(fmt::format("tag{}", target.fiducialId),
                                            target.GetPose().ToPose2d());
  }

  // Call this once just to get rid of the warnings that it is unused.
  // Its a photonlib bug.
  photon::VisionEstimation::EstimateCamPosePNP({}, {}, {}, {}, photon::TargetModel{1_m});
}

void SubVision::Periodic() {
  frc::SmartDashboard::PutNumber("Vision/LastReefTag", _lastReefObservation.reefTag.GetFiducialId());
  if (_lastReefObservation.cameraSide == Side::Left) {
    frc::SmartDashboard::PutString("Vision/Last Used Camera", "Left");
  } else {
    frc::SmartDashboard::PutString("Vision/Last Used Camera", "Right");
  }
  UpdateVision();
}

void SubVision::SimulationPeriodic() {
  _visionSim.Update(SubDrivebase::GetInstance().GetSimPose());
}

void SubVision::UpdateVision() {
  double largestArea = 0;
  std::string leftTargets = "";
  std::string rightTargets = "";

  // Left camera
  std::vector<photon::PhotonPipelineResult> results = _leftCamera.GetAllUnreadResults();
  auto resultCount = results.size();
  if (resultCount > 0) {
    for (auto result : results) {
      _leftEstPose = _leftPoseEstimater.Update(result);
      for (const auto& target : result.targets) {
        leftTargets += std::to_string(target.GetFiducialId()) + ", ";
        if (!CheckReef(target)) {continue;}
        double targetArea = target.GetArea();
        if (targetArea > largestArea ) {
          _lastReefObservation.reefTag = target;
          _lastReefObservation.cameraSide = Side::Left;

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

      for (const auto& target : result.targets) {
        rightTargets += std::to_string(target.GetFiducialId()) + ", ";
        if (!CheckReef(target)) {continue;}
        double targetArea = target.GetArea();
        if (targetArea > largestArea) {
          _lastReefObservation.reefTag = target;
          _lastReefObservation.cameraSide = Side::Right;
          largestArea = targetArea;
        }
      }
    }
  }

  frc::SmartDashboard::PutString("Vision/Left/targets", leftTargets);
  frc::SmartDashboard::PutString("Vision/Right/targets", rightTargets);
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
  int reefTagID = (pose == -1)? _lastReefObservation.reefTag.GetFiducialId() : pose;
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

units::degree_t SubVision::GetReefAlignAngle(Side reefSide) {
  int reefTagID = _lastReefObservation.reefTag.GetFiducialId();
  Side cameraSide = _lastReefObservation.cameraSide;
  if (cameraSide == Left)
  {
    if (reefSide == Left) { return tagToReefAngles[reefTagID].LeftCameraLeftScoreAngle; } //left pole
    else { return tagToReefAngles[reefTagID].LeftCameraRightScoreAngle; } //right pole
  }
  else // CameraSide == Right
  {
    if (reefSide == Left) { return tagToReefAngles[reefTagID].RightCameraLeftScoreAngle; } //left pole
    else { return tagToReefAngles[reefTagID].RightCameraRightScoreAngle; } //right pole
  }

  return 0_deg;
}

units::degree_t SubVision::GetLastReefTagAngle() {
  return _lastReefObservation.reefTag.GetYaw() * 1_deg;
}

double SubVision::GetLastReefTagArea() {
  return _lastReefObservation.reefTag.GetArea();
}

SubVision::Side SubVision::GetLastCameraUsed() {
  return _lastReefObservation.cameraSide;
}

bool SubVision::CheckReef(const photon::PhotonTrackedTarget& reef) {
  const auto& myReef =
      (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) ? redReef : blueReef;
  if (std::find(myReef.begin(), myReef.end(), reef.GetFiducialId()) == myReef.end()) {
    return false;
  }
  units::degree_t errorAngle = SubDrivebase::GetInstance().GetAllianceRelativeGyroAngle().Degrees() -
                               GetReefPose(Left,reef.GetFiducialId()).Rotation().Degrees();

  errorAngle = frc::InputModulus(errorAngle, -180_deg, 180_deg);

  frc::SmartDashboard::PutNumber("Vision/errorAngle", errorAngle.value());
  if ((errorAngle > 30_deg || errorAngle < -30_deg)) {
    return false;
  } else {
    return true;
  }
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

bool SubVision::IsEstimateUsable(photon::EstimatedRobotPose pose) {
  units::meter_t distance = 0_m;
  auto tagCount = pose.targetsUsed.size();
  if (pose.targetsUsed.size() == 0) {
    return 0;
  }
  for (auto target : pose.targetsUsed) {
    distance += target.GetBestCameraToTarget().Translation().Norm();
  }
  distance /= pose.targetsUsed.size();
  return (distance < 0.7_m) || (tagCount > 1);
}