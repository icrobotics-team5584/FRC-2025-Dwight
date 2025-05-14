#include <frc/geometry/Pose2d.h>

namespace ICgeometry { 
    frc::Pose2d ClosestPoseVector(frc::Pose2d startPose, std::vector<frc::Pose2d> poseVector) {
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