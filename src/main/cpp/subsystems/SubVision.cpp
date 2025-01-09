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
  SetDefaultCommand(Run([this] { UpdatePoseEstimator(); })); // Always keep vision on

  _visionSim.AddAprilTags(_tagLayout); // Configure vision sim
  _visionSim.AddCamera(&_cameraSim, _camToBot.Inverse());

  for (auto target : _visionSim.GetVisionTargets()) { //Add AprilTags to field visualization
    SubDrivebase::GetInstance().DisplayPose(fmt::format("tag{}", target.fiducialId),
                                            target.GetPose().ToPose2d());
  }

  // Call this once just to get rid of the warnings that it is unused.
  // Its a photonlib bug.
  photon::VisionEstimation::EstimateCamPosePNP({}, {}, {}, {}, photon::TargetModel{1_m});
}

void SubVision::Periodic() {
  frc::SmartDashboard::PutNumber("Vision/Cam to target x", GetCameraToTarget().X().value());
  frc::SmartDashboard::PutNumber("Vision/Cam to target y", GetCameraToTarget().Y().value());
}

void SubVision::SimulationPeriodic() {
  _visionSim.Update(SubDrivebase::GetInstance().GetPose());
}

void SubVision::UpdatePoseEstimator() {
  auto results = _camera.GetAllUnreadResults();
  auto resultCount = results.size();
  frc::SmartDashboard::PutNumber("Vision/Reuslt count", resultCount);
  if (resultCount == 0) {
    return; // Return if no result, prevent null error later
  }
  std::optional<photon::EstimatedRobotPose> pose;
  for (auto result : results) {
      pose = _robotPoseEstimater.Update(result); // Get estimate pose of robot by vision
      if (result.HasTargets()) {
        _latestTarget = result.GetBestTarget();
      }
  }
  frc::SmartDashboard::PutBoolean("Vision/Has value", pose.has_value());
  if (pose.has_value()) {// Check if pose is vaild
    if (CheckVaild(pose) || frc::RobotBase::IsSimulation()) { // Check if the pose is good and should be referenced
      SubDrivebase::GetInstance().AddVisionMeasurement(pose.value().estimatedPose.ToPose2d(), 0,
                                                       pose.value().timestamp);
      SubDrivebase::GetInstance().DisplayPose("Estimated pose",
                                              pose.value().estimatedPose.ToPose2d()); // Display pose to field visualization
      frc::SmartDashboard::PutNumber("Vision/Estimated robot X",
                                     pose.value().estimatedPose.ToPose2d().X().value()); // Display estimate value to dashboard
      frc::SmartDashboard::PutNumber("Vision/Estimated robot Y",
                                     pose.value().estimatedPose.ToPose2d().Y().value());
    }
  }
}

frc::Translation2d SubVision::GetCameraToTarget() {
  if (_latestTarget == photon::PhotonTrackedTarget()) {return frc::Translation2d(0_m,0_m);}
  frc::Pose2d targetPose = _tagLayout.GetTagPose(_latestTarget.fiducialId).value().ToPose2d();
  frc::Pose2d robotPose = SubDrivebase::GetInstance().GetPose();
  frc::Pose2d relativePose = robotPose.RelativeTo(targetPose);
  return relativePose.Translation();
}

frc::Pose2d SubVision::GetBestTarget() {
 if (_latestTarget == photon::PhotonTrackedTarget()) {return frc::Pose2d(0_m,0_m,0_deg);}
 return _tagLayout.GetTagPose(_latestTarget.fiducialId).value().ToPose2d();
}

bool SubVision::CheckVaild(std::optional<photon::EstimatedRobotPose> pose) {
  const double minArea = 20;
  const double maxArea = 80;
  const double minAmbiguity = 0.2;
  const double maxAmbiguity = 0.8;
  const int desiredTargetLimit = 3;

  auto data = pose.value();

  frc::SmartDashboard::PutNumber("Vision/Target detected", data.targetsUsed.size()); // Display number of target detected

  //Check number of target
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

  //Check area
  if (area < minArea || area > maxArea) {
    return false;
  }

  //Check ambiguity
  if ((ambiguity < minAmbiguity || ambiguity > maxAmbiguity) && !frc::RobotBase::IsSimulation()) {
    return false;
  }

  return true;
}