
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
#include <wpi/interpolating_map.h>

class SubVision : public frc2::SubsystemBase {
public:
  SubVision();
  static SubVision& GetInstance() {
    static SubVision inst;
    return inst;
  }

  void Periodic() override;

  void SimulationPeriodic() override;

  enum Side {
    Left = 1,
    Right = 2
  };

  /**
   * Update pose estimater with vision, should be called every frame
   */
  void UpdateVision();

  units::degree_t GetLastReefTagAngle();
  double GetLastReefTagArea();
  frc::Pose2d GetReefPose(Side side, int pose);
  units::degree_t GetReefAlignAngle(Side reefSide);

  std::map<Side, std::optional<photon::EstimatedRobotPose>> GetPose();

  frc::Pose2d GetSourcePose(int tagId);

  double GetDev(photon::EstimatedRobotPose pose);

 private:
 /**
 * Check if the pose if good enough to be used as reference
 * 
 * @param pose Pose of the target
 */
  bool CheckReef(const photon::PhotonTrackedTarget& reef);
  struct ReefObservation {
    photon::PhotonTrackedTarget reefTag;
    Side cameraSide;
  };
  struct ReefObservation _lastReefObservation;

  std::string _tagLayoutPath = frc::filesystem::GetDeployDirectory() + "/2025-reefscape.json";
  frc::AprilTagFieldLayout _tagLayout{_tagLayoutPath};

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

  struct ReefCameraAngles {
    units::degree_t RightCameraLeftScoreAngle;
    units::degree_t RightCameraRightScoreAngle;
    units::degree_t LeftCameraLeftScoreAngle;
    units::degree_t LeftCameraRightScoreAngle;
  };

  std::map<int, ReefCameraAngles> tagToReefAngles {
    // left reef
    //    right cam   |        left cam
    {17, {0_deg, 0_deg, 11.4_deg, -8.300_deg}},
    {18, {0_deg, 0_deg, 11.4_deg, -8.300_deg}},
    {19, {0_deg, 0_deg, 11.4_deg, -8.300_deg}},
    {20, {0_deg, 0_deg, 11.4_deg, -8.300_deg}},
    {21, {0_deg, 0_deg, 11.4_deg, -8.300_deg}},
    {22, {0_deg, 0_deg, 11.4_deg, -8.300_deg}},

    // red reef
    //    right cam   |        left cam
    { 6, {0_deg, 0_deg, 11.4_deg, -8.300_deg}},
    { 7, {0_deg, 0_deg, 11.4_deg, -8.300_deg}},
    { 8, {0_deg, 0_deg, 11.4_deg, -8.300_deg}},
    { 9, {0_deg, 0_deg, 11.4_deg, -8.300_deg}},
    {10, {0_deg, 0_deg, 11.4_deg, -8.300_deg}},
    {11, {0_deg, 0_deg, 11.4_deg, -8.300_deg}},

};

std::map<int, ReefPositions> tagToReefPositions = {
    {17, {60_deg-90_deg, 3.529_m, 2.805_m, 3.820_m, 2.671_m}},
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

  //Left camera config
  std::string _leftCamName = "photonvision_5584";

  photon::PhotonCamera _leftCamera{_leftCamName};

  photon::PhotonCameraSim _leftCamSim{&_leftCamera};
  photon::VisionSystemSim _leftVisionSim{_leftCamName};

  frc::Transform3d _leftBotToCam{{270_mm,305_mm,220_mm},{0_deg,0_deg,320_deg}};

  photon::PhotonPoseEstimator _leftPoseEstimater{
    _tagLayout,
    photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
    _leftBotToCam
  };

  std::optional<photon::EstimatedRobotPose> _leftEstPose;

  //Right camera config
  std::string _rightCamName = "placeholder"; // Need to find out

  photon::PhotonCamera _rightCamera{_rightCamName};

  photon::PhotonCameraSim _rightCamSim{&_rightCamera};
  photon::VisionSystemSim _rightVisionSim{_rightCamName};

  frc::Transform3d _rightBotToCam{{270_mm,-230_mm,220_mm},{0_deg,0_deg,40_deg}};

  photon::PhotonPoseEstimator _rightPoseEstimater{
    _tagLayout,
    photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
    _rightBotToCam
  };

  std::optional<photon::EstimatedRobotPose> _rightEstPose;

  wpi::interpolating_map<units::meter_t, double> _devTable;
};

