#pragma once
#include <frc2/command/Commands.h>
#include <frc2/command/button/CommandXboxController.h>

namespace cmd {
    frc2::CommandPtr IntakeFullSequence();
    frc2::CommandPtr RemoveAlgaeLow();
    frc2::CommandPtr RemoveAlgaeHigh();
    frc2::CommandPtr IntakeFromSource();
    frc2::CommandPtr Outtake();
    frc2::CommandPtr SetL1();
    frc2::CommandPtr SetL2();
    frc2::CommandPtr SetL3();
    frc2::CommandPtr SetL4();
}