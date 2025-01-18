// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/VisionCommand.h"
#include "subsystems/SubVision.h"
#include "subsystems/SubDriveBase.h"
#include <frc2/command/Commands.h>
#include <frc/DriverStation.h>
namespace cmd {
using namespace frc2::cmd;
/**
 * Command to align to Apriltag with y offset (to match left or right of reef)
 * @param side align to left: 1, align to right: 2
 * @return A command that will align to the tag when executed.
 */
frc2::CommandPtr YAlignWithTarget(int side, frc2::CommandXboxController& controller) {
  return SubDrivebase::GetInstance().Drive(
      [side, &controller] {
        frc::ChassisSpeeds speeds = SubDrivebase::GetInstance().CalcDriveToPoseSpeeds(
            SubVision::GetInstance().GetReefPose(side));
        return speeds;
      },
      true);
}

frc2::CommandPtr AddVisionMeasurement() {
  return Run(
      [] {
        auto estimatedPose = SubVision::GetInstance().GetPose();
        frc::SmartDashboard::PutBoolean("Vision/is teleop", frc::DriverStation::IsTeleop());
        frc::SmartDashboard::PutBoolean("Vision/has value", estimatedPose.has_value());
        if (estimatedPose.has_value() && frc::DriverStation::IsTeleop()) {
          auto estimatedPoseValue = estimatedPose.value();
          SubDrivebase::GetInstance().AddVisionMeasurement(
              estimatedPoseValue.estimatedPose.ToPose2d(), 0, estimatedPoseValue.timestamp);
          SubDrivebase::GetInstance().DisplayPose("EstimatedPose",
                                                  estimatedPoseValue.estimatedPose.ToPose2d());
        } else {
          SubDrivebase::GetInstance().DisplayPose("EstimatedPose", {});
        }
      },
      {&SubVision::GetInstance()});
}  // namespace cmd

// check pose -> decide which source is closer -> drive there
frc2::CommandPtr AlignToSource(frc2::CommandXboxController& controller) {
  return SubDrivebase::GetInstance().Drive(
      [&controller] {
        frc::Pose2d sourcePose = {0_m, 0_m, 0_deg};
        units::meter_t poseY = SubDrivebase::GetInstance().GetPose().Y();
        if (poseY > 4.025_m) {
          if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) {
            sourcePose = SubVision::GetInstance().GetSourcePose(2);
          } else {
            sourcePose = SubVision::GetInstance().GetSourcePose(13);
          }
        } else {
          if (frc::DriverStation::GetAlliance() == frc::DriverStation::kRed) {
            sourcePose = SubVision::GetInstance().GetSourcePose(1);
          } else {
            sourcePose = SubVision::GetInstance().GetSourcePose(12);
          }
        }
        return SubDrivebase::GetInstance().CalcDriveToPoseSpeeds(sourcePose);
      },
      true);
}

frc2::CommandPtr rotateToReef(frc2::CommandXboxController& controller){
  return SubDrivebase::GetInstance().Drive(
    [&controller] {
    
    units::degree_t targetAngle = SubVision::GetInstance().getLastReefIdAngle();
    units::degree_t currentAngle = SubDrivebase::GetInstance().GetPose().Rotation().Degrees();
    units::turns_per_second_t rotSpeed = SubDrivebase::GetInstance().CalcRotateSpeed(currentAngle - targetAngle);

    frc::ChassisSpeeds joystickSpeeds = SubDrivebase::GetInstance().CalcJoystickSpeeds(controller);

    frc::ChassisSpeeds speeds;
    speeds.vx = joystickSpeeds.vx;
    speeds.vy = joystickSpeeds.vy;
    speeds.omega = rotSpeed;

    return speeds;
  }, true);

} 
}  // namespace cmd