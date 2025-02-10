#pragma once
#include <frc2/command/Commands.h>
#include <frc2/command/button/CommandXboxController.h>

namespace cmd {
    frc2::CommandPtr IntakeFullSequence();
    frc2::CommandPtr ClimbEngageSequence();
    frc2::CommandPtr ClimbDisengageSequence();
    frc2::CommandPtr RemoveAlgae();
}