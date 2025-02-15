#pragma once
#include <frc2/command/Commands.h>
#include <frc2/command/button/CommandXboxController.h>

namespace cmd {
    frc2::CommandPtr IntakeFullSequence();
    frc2::CommandPtr ClimbUpSequence();
    frc2::CommandPtr ClimbDownSequence();
    frc2::CommandPtr RemoveAlgae();
}