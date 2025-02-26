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
  frc::SmartDashboard::PutNumber("Vision/LastReefTag", _lastReefObservation.reefTag.GetFiducialId());
  if (_lastReefObservation.cameraSide == Side::Left) {
    frc::SmartDashboard::PutString("Vision/Last Used Camera", "Left");
  } else {
    frc::SmartDashboard::PutString("Vision/Last Used Camera", "Right");
  }
  UpdateVision();
}

void SubVision::SimulationPeriodic() {
  _leftVisionSim.Update(SubDrivebase::GetInstance().GetSimPose());
}

void SubVision::UpdateVision() {
  double largestArea = 2;

  // Left camera
  std::vector<photon::PhotonPipelineResult> results = _leftCamera.GetAllUnreadResults();
  auto resultCount = results.size();
  if (resultCount > 0) {
    for (auto result : results) {
      _leftEstPose = _leftPoseEstimater.Update(result);
      if (_leftEstPose.value().timestamp - _lastReefObservation.timestamp >= 10_s) {
        _lastReefObservation.timestamp = -1_s;
      }
      frc::SmartDashboard::PutNumber("Vision/Left/target", result.GetBestTarget().fiducialId);

      for (const auto& target : result.targets) {
        if (!CheckReef(target)) {continue;}
        double targetArea = target.GetArea();
        if (targetArea > largestArea ) {
          _lastReefObservation.reefTag = target;
          _lastReefObservation.cameraSide = Side::Left;
          _lastReefObservation.timestamp = _leftEstPose.value().timestamp;

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
        if (!CheckReef(target)) {continue;}
        double targetArea = target.GetArea();
        if (targetArea > largestArea) {
          _lastReefObservation.reefTag = target;
          _lastReefObservation.cameraSide = Side::Right;
          _lastReefObservation.timestamp = _rightEstPose.value().timestamp;

          largestArea = targetArea;
        }
      }
    }
  }
}

bool SubVision::HadReef() {
  return _lastReefObservation.timestamp == -1_s;
}

std::map<SubVision::Side, std::optional<photon::EstimatedRobotPose>> SubVision::GetPose() {
  return {{Left, _leftEstPose}, {Right, _rightEstPose}};
}

frc::Pose2d SubVision::GetSourcePose(int tagId) {
  return tagToSourcePose[tagId];
}

frc::Pose2d SubVision::GetReefPose(int pose, Side side) {
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

frc::Pose2d SubVision::GetLastReefPose(Side side) {
  return GetReefPose(_lastReefObservation.reefTag.GetFiducialId(),side);
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
                               GetReefPose(reef.GetFiducialId(),Left).Rotation().Degrees();

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
