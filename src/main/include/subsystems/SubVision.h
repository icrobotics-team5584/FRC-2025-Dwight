// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose3d.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <photon/PhotonCamera.h>
#include <photon/simulation/VisionSystemSim.h>
#include <photon/PhotonPoseEstimator.h>

class SubVision : public frc2::SubsystemBase {
 public:
  SubVision();
  static SubVision& GetInstance() {
    static SubVision inst;
    return inst;
  }

  enum FieldElement { SPEAKER, AMP, SPEAKER_SIDE, SOURCE_LEFT, SOURCE_RIGHT };

  void Periodic() override;
  void SimulationPeriodic() override;
  frc::Pose3d GetTagPose(int id);
  int FindID(FieldElement chosenFieldElement);
  void UpdatePoseEstimator();
  frc2::CommandPtr CmdUpdatePoseEstimator();

 private:
  bool CheckVaild(std::optional<photon::EstimatedRobotPose> pose);

  std::string _cameraName = "photonvision_5584";//"arducam";
  photon::PhotonCamera _camera{_cameraName};
  photon::PhotonCameraSim _cameraSim{&_camera};

  frc::Transform3d _camToBot{{0_mm, -200_mm, -150_mm}, {}};//{0_deg,0_deg,180_deg}};

  frc::AprilTagFieldLayout tagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField);

  std::map<FieldElement, int> blueFieldElement = {
      {SPEAKER, 7}, {SPEAKER_SIDE, 8}, {AMP, 6}, {SOURCE_LEFT, 2}, {SOURCE_RIGHT, 1}};
  std::map<FieldElement, int> redFieldElement = {
      {SPEAKER, 4}, {SPEAKER_SIDE, 3}, {AMP, 5}, {SOURCE_LEFT, 10}, {SOURCE_RIGHT, 9}};

  photon::VisionSystemSim _visionSim{_cameraName};

  photon::PhotonPoseEstimator robotPoseEstimater{
      tagLayout,
      photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
      _camToBot.Inverse()
  };
};
