
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
  Side GetLastCameraUsed();
  frc::Pose2d GetReefPose(Side side, int pose);
  std::vector<std::pair<frc::Pose2d, frc::Pose2d>> GetBlueReefPoses(void);
  std::vector<std::pair<frc::Pose2d, frc::Pose2d>> GetRedReefPoses(void);
  units::degree_t GetReefAlignAngle(Side reefSide);

  std::map<Side, std::optional<photon::EstimatedRobotPose>> GetPose();

  frc::Pose2d GetSourcePose(int tagId);

  double GetDev(photon::EstimatedRobotPose pose);

  bool IsEstimateUsable(photon::EstimatedRobotPose pose);

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
    // blue reef
    //    right cam   |        left cam
    {17, {0_deg, -21.45_deg , 19.2_deg, 0_deg}},
    {18, {0_deg, -21.0_deg , 19.52_deg, 0_deg}},
    {19, {0_deg, -21.87_deg , 19.27_deg, 0_deg}},
    {20, {0_deg, -21.11_deg , 18.83_deg, 0_deg}},
    {21, {0_deg, -21.32_deg , 19.48_deg, 0_deg}},
    {22, {0_deg, -21.45_deg , 18.96_deg, 0_deg}},

    // red reef
    //    right cam   |        left cam
    { 6, {0_deg, -21.17_deg , 19.3_deg, 0_deg}},
    { 7, {0_deg, -20.90_deg , 20.15_deg, 0_deg}},
    { 8, {0_deg, -21.84_deg , 18.9_deg, 0_deg}},
    { 9, {0_deg, -20.73_deg , 18.8_deg, 0_deg}},
    {10, {0_deg, -21.55_deg , 19.02_deg, 0_deg}},
    {11, {0_deg, -21.65_deg , 18.24_deg, 0_deg}},

};

std::map<int, ReefPositions> tagToReefPositions = {
    {17, {60_deg-90_deg, 3.25_m, 2.98_m, 3.99_m, 2.84_m}},
    {18, {0_deg-90_deg, 3.22_m, 4.16_m, 3.22_m, 3.89_m}},
    {19, {300_deg-90_deg, 3.96_m, 5.19_m, 3.720_m, 5.050_m}},
    {20, {240_deg-90_deg, 5.220_m, 5.07_m, 5.0_m, 5.20_m}},
    {21, {180_deg-90_deg, 5.760_m, 3.90_m, 5.760_m, 4.170_m}},
    {22, {120_deg-90_deg, 5.030_m, 2.870_m, 5.250_m, 3.00_m}},

    {6, {120_deg-90_deg, 13.590_m, 2.860_m, 13.820_m, 3.0_m}},
    {7, {180_deg-90_deg, 14.330_m, 4.210_m, 14.33_m, 4.170_m}},
    {8, {240_deg-90_deg, 13.790_m, 5.07_m, 13.560_m, 5.20_m}},
    {9, {300_deg-90_deg, 12.530_m, 5.190_m, 12.30_m, 5.05_m}},
    {10, {0_deg-90_deg, 11.79_m, 4.15_m, 11.79_m, 2.850_m}},
    {11, {60_deg-90_deg, 12.330_m, 2.970_m, 12.550_m, 2.850_m}}

};
  //+9.4418

  //Left camera config
  std::string _leftCamName = "ICR_OV2981_L (1)";

  photon::PhotonCamera _leftCamera{_leftCamName};

  photon::PhotonCameraSim _leftCamSim{&_leftCamera};
  photon::VisionSystemSim _visionSim{_leftCamName};

  frc::Transform3d _leftBotToCam{{-270_mm,270_mm,220_mm},{0_deg,5_deg,45_deg}};

  photon::PhotonPoseEstimator _leftPoseEstimater{
    _tagLayout,
    photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
    _leftBotToCam
  };

  std::optional<photon::EstimatedRobotPose> _leftEstPose;

  //Right camera config
  std::string _rightCamName = "ICR_OV9281_R (1)"; // Need to find out

  photon::PhotonCamera _rightCamera{_rightCamName};

  photon::PhotonCameraSim _rightCamSim{&_rightCamera};

  frc::Transform3d _rightBotToCam{{270_mm,270_mm,220_mm},{0_deg,5_deg,135_deg}};

  photon::PhotonPoseEstimator _rightPoseEstimater{
    _tagLayout,
    photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR,
    _rightBotToCam
  };

  std::optional<photon::EstimatedRobotPose> _rightEstPose;

  wpi::interpolating_map<units::meter_t, double> _devTable;
};
