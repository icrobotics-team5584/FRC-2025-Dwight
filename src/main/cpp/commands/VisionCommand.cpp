// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/VisionCommand.h"
#include "subsystems/SubVision.h"
#include "subsystems/SubDriveBase.h"
#include <frc2/command/Commands.h>

namespace cmd {
using namespace frc2::cmd;
frc2::CommandPtr YAlignWithTarget(units::meter_t offset) {
  return SubDrivebase::GetInstance()
      .Drive(
          [offset]() {
            return frc::ChassisSpeeds(
                0_mps,
                0.01_mps * pow((SubVision::GetInstance().GetCameraToTarget().Y() - offset).value(),5),
                0_tps);
          },
          false)
      .Until([offset] { return SubVision::GetInstance().GetCameraToTarget().Y() - offset < 0.02_m && SubVision::GetInstance().GetCameraToTarget().Y() - offset > -0.02_m; });
}
}  // namespace cmd