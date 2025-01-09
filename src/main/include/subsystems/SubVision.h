// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>
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

  void Periodic() override;

  void SimulationPeriodic() override;

  /**
 * Update pose estimater with vision, should be called every frame
 */
  void UpdatePoseEstimator();

  /**
   *  Get transformation from camera to current target
   */
  frc::Translation2d GetCameraToTarget();

 private:
 /**
 * Check if the pose if good enough to be used as reference
 * 
 * @param pose Pose of the target
 */
  bool CheckVaild(std::optional<photon::EstimatedRobotPose> pose);

  std::string _cameraName = "photonvision_5584";
  photon::PhotonCamera _camera{_cameraName};
  photon::PhotonCameraSim _cameraSim{&_camera}; // For simulation

  frc::Transform3d _camToBot{{0_mm, 0_mm, -150_mm}, {}};

  frc::AprilTagFieldLayout _tagLayout = frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField);

  photon::VisionSystemSim _visionSim{_cameraName};

  photon::PhotonPoseEstimator _robotPoseEstimater{
      _tagLayout,
      photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
      _camToBot.Inverse()
  };

  photon::PhotonTrackedTarget _latestTarget = photon::PhotonTrackedTarget();
};
