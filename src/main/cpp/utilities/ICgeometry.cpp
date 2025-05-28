#include "utilities/ICgeometry.h"
#include <cmath>
#include <units/length.h>
#include <units/angle.h>
#include <frc/geometry/Pose2d.h>

namespace ICgeometry { 
    frc::Pose2d xFilpPose(frc::Pose2d pose) {
        units::meter_t xdiff = units::math::abs(FIELD_LENGTH/2 - pose.Y());
        units::meter_t x = (pose.X() < FIELD_LENGTH/2) ? FIELD_LENGTH/2 + xdiff : FIELD_LENGTH/2 - xdiff;
        return frc::Pose2d(x, pose.Y(), pose.Rotation());
    }

    frc::Pose2d yFilpPose(frc::Pose2d pose) {
        units::meter_t ydiff = units::math::abs(FIELD_WIDTH/2 - pose.Y());
        units::meter_t y = (pose.Y() < FIELD_WIDTH/2) ? FIELD_WIDTH/2 + ydiff : FIELD_WIDTH/2 - ydiff;
        return frc::Pose2d(pose.X(), y, pose.Rotation());
    }

    frc::Pose2d xyFilpPose(frc::Pose2d pose) {
        return yFilpPose(xFilpPose(pose));
    }

    frc::Rotation2d PoseDirection(frc::Pose2d origin, frc::Pose2d destination) {
        double x = (origin.X() - destination.X()).value();
        double y = (origin.Y() - destination.Y()).value();
        return frc::Rotation2d(static_cast<units::angle::radian_t>(std::tan(y/x)));
    }
}