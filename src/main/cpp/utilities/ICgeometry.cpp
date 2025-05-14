#include "utilities/ICgeometry.h"
#include <units/length.h>
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

    frc::Pose2d closestPoseVector(frc::Pose2d startPose, std::vector<frc::Pose2d> poseVector) {
        if (poseVector.empty()) { return startPose; }
        double shortestDistance = 1024.0;
        double curDistance;
        int closestIndex;
        for (int i = 0; i < poseVector.size(); i++) {
            curDistance = startPose.Translation().Distance(poseVector.at(i).Translation()).value();
            if (curDistance < shortestDistance) {
                shortestDistance = curDistance;
                closestIndex = i;
            }
        }
        return poseVector.at(closestIndex);
    }
}