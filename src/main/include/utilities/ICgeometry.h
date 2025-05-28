#include <frc/geometry/Pose2d.h>

namespace ICgeometry { 
    const units::meter_t FIELD_WIDTH = 8.05_m;
    const units::meter_t FIELD_LENGTH = 17.55_m;
        
    frc::Pose2d xPoseFlip(frc::Pose2d pose);
    frc::Pose2d yPoseFlip(frc::Pose2d pose);
    frc::Pose2d xyPoseFlip(frc::Pose2d pose);

    frc::Rotation2d PoseDirection(frc::Pose2d origin, frc::Pose2d destination);
}