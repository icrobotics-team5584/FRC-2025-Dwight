// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <cmath>

#include <wpi/SymbolExports.h>
#include <wpi/array.h>

#include "utilities/ICPoseEstimator.h"
#include "frc/geometry/Pose2d.h"
#include "frc/geometry/Rotation2d.h"
#include "frc/kinematics/SwerveDriveKinematics.h"
#include "frc/kinematics/SwerveDriveOdometry.h"
#include "units/time.h"


/**
 * This class wraps Swerve Drive Odometry to fuse latency-compensated
 * vision measurements with swerve drive encoder distance measurements. It is
 * intended to be a drop-in for SwerveDriveOdometry.
 *
 * Update() should be called every robot loop.
 *
 * AddVisionMeasurement() can be called as infrequently as you want; if you
 * never call it, then this class will behave as regular encoder
 * odometry.
 */
// language: cpp
// filepath: /c:/src/FRC-2025-Dwight/src/main/include/utilities/ICSwerveDrivePoseEstimator.h
// ...existing code...
template <size_t NumModules>
class ICSwerveDrivePoseEstimator
    : public ic::ICPoseEstimator<wpi::array<frc::SwerveModuleState, NumModules>,
                           wpi::array<frc::SwerveModulePosition, NumModules>> {
 public:
  /**
   * Constructs a ICSwerveDrivePoseEstimator with default standard deviations
   * for the model and vision measurements.
   *
   * The default standard deviations of the model states are
   * 0.1 meters for x, 0.1 meters for y, and 0.1 radians for heading.
   * The default standard deviations of the vision measurements are
   * 0.9 meters for x, 0.9 meters for y, and 0.9 radians for heading.
   *
   * @param kinematics A correctly-configured kinematics object for your
   *     drivetrain.
   * @param gyroAngle The current gyro angle.
   * @param modulePositions The current distance and rotation measurements of
   *     the swerve modules.
   * @param initialPose The starting pose estimate.
   */
  ICSwerveDrivePoseEstimator(
      frc::SwerveDriveKinematics<NumModules>& kinematics,
      const frc::Rotation2d& gyroAngle,
      const wpi::array<frc::SwerveModulePosition, NumModules>& modulePositions,
      const frc::Pose2d& initialPose)
      : ICSwerveDrivePoseEstimator{kinematics,      gyroAngle,
                                 modulePositions, initialPose,
                                 wpi::array<double, 3>{0.1, 0.1, 0.1},
                                 wpi::array<double, 3>{0.9, 0.9, 0.9}} {}

  /**
   * Constructs a ICSwerveDrivePoseEstimator.
   *
   * @param kinematics A correctly-configured kinematics object for your
   *     drivetrain.
   * @param gyroAngle The current gyro angle.
   * @param modulePositions The current distance and rotation measurements of
   *     the swerve modules.
   * @param initialPose The starting pose estimate.
   * @param stateStdDevs Standard deviations of the pose estimate (x position in
   *     meters, y position in meters, and heading in radians). Increase these
   *     numbers to trust your state estimate less.
   * @param visionMeasurementStdDevs Standard deviations of the vision pose
   *     measurement (x position in meters, y position in meters, and heading in
   *     radians). Increase these numbers to trust the vision pose measurement
   *     less.
   */
  ICSwerveDrivePoseEstimator(
      frc::SwerveDriveKinematics<NumModules>& kinematics,
      const frc::Rotation2d& gyroAngle,
      const wpi::array<frc::SwerveModulePosition, NumModules>& modulePositions,
      const frc::Pose2d& initialPose, const wpi::array<double, 3>& stateStdDevs,
      const wpi::array<double, 3>& visionMeasurementStdDevs)
      : ICSwerveDrivePoseEstimator::ICPoseEstimator(kinematics, m_odometryImpl, stateStdDevs,
                      visionMeasurementStdDevs),
        m_odometryImpl{kinematics, gyroAngle, modulePositions, initialPose} {
    this->ResetPose(initialPose);
  }

 private:
  frc::SwerveDriveOdometry<NumModules> m_odometryImpl;
};