// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubVision.h"
#include "subsystems/SubDrivebase.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>

SubVision::SubVision() {
    //cameraToRobotTransform
    SetDefaultCommand(CmdUpdatePoseEstimator());
    visionSim.AddAprilTags(tagLayout);
    for (auto target : visionSim.GetVisionTargets()) {
        SubDrivebase::GetInstance().DisplayPose(fmt::format("tag{}", target.fiducialId), target.GetPose().ToPose2d());
    }
}

// This method will be called once per scheduler run
void SubVision::Periodic() {
  // UpdatePoseEstimator();
  
}

void SubVision::SimulationPeriodic() {
    visionSim.Update(SubDrivebase::GetInstance().GetPose());
}

void SubVision::UpdatePoseEstimator() {
  auto result = camera.GetLatestResult();
  auto pose = robotPoseEstimater.Update(result);
  frc::SmartDashboard::PutNumber("Vision/Reference pose X", robotPoseEstimater.GetReferencePose().X().value());
  frc::SmartDashboard::PutNumber("Vision/Reference pose Y", robotPoseEstimater.GetReferencePose().Y().value());
  frc::SmartDashboard::PutNumber("Vision/Reference pose Z", robotPoseEstimater.GetReferencePose().Z().value());
  if (pose.has_value())
  {
    if (CheckVaild(pose)) {
    SubDrivebase::GetInstance().AddVisionMeasurement(
        pose.value().estimatedPose.ToPose2d(),
        0,
        pose.value().timestamp);
    SubDrivebase::GetInstance().DisplayPose("Estimated pose", pose.value().estimatedPose.ToPose2d());
    frc::SmartDashboard::PutNumber("Vision/Estimated robot X", pose.value().estimatedPose.ToPose2d().X().value());
    frc::SmartDashboard::PutNumber("Vision/Estimated robot Y", pose.value().estimatedPose.ToPose2d().Y().value());
    }
  }
  frc::SmartDashboard::PutBoolean("Vision/Has value", pose.has_value());
}

frc2::CommandPtr SubVision::CmdUpdatePoseEstimator() {
    return Run(
        [this] {
          UpdatePoseEstimator();
        }
    );
}

frc::Pose3d SubVision::GetTagPose(int id) {
    return tagLayout.GetTagPose(id).value();
}

int SubVision::FindID(FieldElement chosenFieldElement) {
  if (auto ally = frc::DriverStation::GetAlliance()) {
    if (ally.value() == frc::DriverStation::Alliance::kBlue) {
      return blueFieldElement[chosenFieldElement];
    }

    if (ally.value() == frc::DriverStation::Alliance::kRed) {
      return redFieldElement[chosenFieldElement];
    }
  }

  return redFieldElement[chosenFieldElement];
}

bool SubVision::CheckVaild(std::optional<photon::EstimatedRobotPose> pose) {
  const double minArea = 20;
  const double maxArea = 80;
  const double minAmbiguity = 0.2;
  const double maxAmbiguity = 0.8;
  const int desiredTargetLimit = 3;

  auto data = pose.value();

  frc::SmartDashboard::PutNumber("Vision/Target detected", data.targetsUsed.size());

  if (data.targetsUsed.size() > desiredTargetLimit) {
    return false;
  }

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

  if (area < minArea || area > maxArea) {
    return false;
  }

  if ((ambiguity < minAmbiguity || ambiguity > maxAmbiguity) && !frc::RobotBase::IsSimulation()) {
    return false;
  }

  return true;
}

// Checklist: 

// Area
// Length
// Num of tags
// Ambigurity