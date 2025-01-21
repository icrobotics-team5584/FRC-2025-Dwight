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
#include <frc/Filesystem.h>

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
  void UpdatePoseEstimator(std::vector<photon::PhotonPipelineResult> results);
  void UpdateLatestTags(std::vector<photon::PhotonPipelineResult> results);

  frc::Pose2d GetReefPose(int side);

  std::optional<photon::EstimatedRobotPose> GetPose();

  frc::Pose2d GetSourcePose(int tagId);




 private:
 /**
 * Check if the pose if good enough to be used as reference
 * 
 * @param pose Pose of the target
 */
  bool CheckValid(std::optional<photon::EstimatedRobotPose> pose);

  std::optional<photon::EstimatedRobotPose> _pose;

  std::map<int, frc::Pose2d> tagToSourcePose = {
    {13, {1.621_m, 7.378_m, 324_deg}},
    {12, {1.621_m, 0.686_m, 36_deg}},
    {1, {16.981_m, 0.686_m, 324_deg}},
    {2, {16.981_m, 7.378_m, 36_deg}},
  };

  std::array<int, 6> blueReef = {
    18, 19, 17, 21, 20, 22
  };

  std::array<int, 6> redReef = {
    7, 8, 6, 10, 9, 11
  };

  struct ReefPositions {
    units::degree_t angle;
    units::meter_t leftX;
    units::meter_t leftY;
    units::meter_t rightX;
    units::meter_t rightY;
  };

std::map<int, ReefPositions> tagToReefPositions = {
    {17, {60_deg-90_deg, 3.529_m, 2.805_m, 3.820_m, 2.671_m}}, //{17, {0_deg, 3.470_m, 2_m, 3.470_m, 2_m}},
    {18, {0_deg-90_deg, 3.150_m, 4.190_m, 3.150_m, 3.870_m}},
    {19, {300_deg-90_deg, 3.960_m, 5.250_m, 3.690_m, 5.090_m}},
    {20, {240_deg-90_deg, 5.290_m, 5.095_m, 5.020_m, 5.250_m}},
    {21, {180_deg-90_deg, 5.810_m, 3.860_m, 5.810_m, 4.190_m}},
    {22, {120_deg-90_deg, 5.010_m, 2.800_m, 5.290_m, 2.950_m}},

    {6, {120_deg-90_deg, 13.410_m, 3.100_m, 13.700_m, 3.260_m}},
    {7, {180_deg-90_deg, 14.060_m, 3.860_m, 14.060_m, 4.190_m}},
    {8, {240_deg-90_deg, 13.700_m, 4.800_m, 13.420_m, 4.980_m}},
    {9, {300_deg-90_deg, 12.710_m, 4.970_m, 12.430_m, 4.800_m}},
    {10, {0_deg-90_deg, 4.190_m, 12.080_m, 3.860_m, 12.080_m}},
    {11, {60_deg-90_deg, 12.430_m, 3.260_m, 12.720_m, 3.100_m}}
  };

  //+9.4418
  photon::PhotonTrackedTarget _lastReefTag;
  std::string _cameraName = "photonvision_5584";

  photon::PhotonCamera _camera{_cameraName};
  photon::PhotonCameraSim _cameraSim{&_camera}; // For simulation

  frc::Transform3d _botToCam{{300_mm, 300_mm, 200_mm}, {0_deg, 0_deg, 117_deg}};
  std::string _tagLayoutPath = frc::filesystem::GetDeployDirectory() + "/2025-reefscape.json";
  frc::AprilTagFieldLayout _tagLayout{_tagLayoutPath};

  photon::VisionSystemSim _visionSim{_cameraName};

  photon::PhotonPoseEstimator _robotPoseEstimater{
    _tagLayout,
    photon::PoseStrategy::LOWEST_AMBIGUITY,
    // Change to: photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
    _botToCam
  };

  photon::PhotonTrackedTarget _latestTarget = photon::PhotonTrackedTarget();
};

