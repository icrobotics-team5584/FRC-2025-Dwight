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
  //Reef close middle, Reef close left, Reef close right, Reef far middle, Reef far left, Reef far right, Source left, Source right, Processor, Barge close, Barge far
  enum FieldElement {SourceL, SourceR, Processor, BargeC, BargeF};

  void Periodic() override;

  void SimulationPeriodic() override;

  /**
 * Update pose estimater with vision, should be called every frame
 */
  void UpdatePoseEstimator();

  frc::Pose2d GetReefPose(int side);


  std::optional<photon::EstimatedRobotPose> GetPose();

  /**
   *  Get transformation from camera to current target
   */
  frc::Translation2d GetCameraToTarget();

  frc::Pose2d GetBestTarget();




 private:
 /**
 * Check if the pose if good enough to be used as reference
 * 
 * @param pose Pose of the target
 */
  bool CheckValid(std::optional<photon::EstimatedRobotPose> pose);

  std::optional<photon::EstimatedRobotPose> _pose;

  std::map<FieldElement, int> blueFieldelement = {
    {SourceL, 13}, {SourceR, 12}, {Processor, 16}, {BargeC, 14}, {BargeF, 4}
  };

  std::array<int, 6> blueReef = {
    18, 19, 17, 21, 20, 22
  };

  std::map<FieldElement, int> redFieldElement = {
    {SourceL, 1}, {SourceR, 2}, {Processor, 3}, {BargeC, 5}, {BargeF, 15}
  };

  std::array<int, 6> redReef = {
    7, 8, 6, 10, 9, 11
  };

  std::map<int, units::degree_t> reefAngles = {
    {18, 0_deg}, {17, 60_deg}, {22, 120_deg}, {21, 180_deg}, {20, 240_deg}, {19, 300_deg},
    {10, 0_deg}, {11, 60_deg}, {6, 120_deg}, {7, 180_deg}, {8, 240_deg}, {9, 300_deg}
  };


//This is a map of int to pair<pair, pair.> The int is the fiducial ID. The inner pairs are the x and y position of the tag. 
//The outer pair is to control the side of the reef you are aligning to. Formatted L, R
  std::map<int, std::pair<std::pair<units::meter_t, units::meter_t>, std::pair<units::meter_t, units::meter_t>>> reefPositions = {
    {18, {{2.829_m, 4.112_m}, {2.828_m, 3.861_m}}},
    {19, {{3.810_m, 5.160_m}, {3.560_m, 5.382_m}}},
    {17, {{3.522_m, 2.697_m}, {3.820_m, 2.514_m}}},
    {22, {{5.407_m, 2.639_m}, {5.176_m, 2.476_m}}},
    {21, {{6.139_m, 4.150_m},{6.139_m, 3.852_m}}},
    {20, {{5.176_m, 5.526_m},{5.417_m, 5.401_m}}},
    {10, {{12.2708_m, 4.112_m},{12.2698_m, 3.861_m}}},
    {9, {{13.2818_m, 5.160_m}, {13.0018_m, 5.382_m}}},
    {11, {{12.9638_m, 2.697_m}, {13.2618_m, 2.514_m}}},
    {6, {{14.8488_m, 2.639_m},{14.6178_m, 2.476_m}}},
    {7, {{15.5808_m, 4.150_m},{15.5808_m, 3.852_m}}},
    {8, {{14.6178_m, 5.526_m},{14.8588_m, 5.401_m}}},
    };
//+9.4418
  photon::PhotonTrackedTarget _lastReefTag;
  std::string _cameraName = "photonvision_5584";
  photon::PhotonCamera _camera{_cameraName};
  photon::PhotonCameraSim _cameraSim{&_camera}; // For simulation

  frc::Transform3d _camToBot{{0_mm, 0_mm, 0_mm}, {}};
  std::string _tagLayoutPath = "src/main/deploy/2025-reefscape.json";
  frc::AprilTagFieldLayout _tagLayout{_tagLayoutPath};

  photon::VisionSystemSim _visionSim{_cameraName};

  photon::PhotonPoseEstimator _robotPoseEstimater{
      _tagLayout,
      photon::PoseStrategy::LOWEST_AMBIGUITY,
      // Change to: photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
      _camToBot.Inverse()
  };

  photon::PhotonTrackedTarget _latestTarget = photon::PhotonTrackedTarget();
};
