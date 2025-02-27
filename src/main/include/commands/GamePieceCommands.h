#pragma once
#include <frc2/command/Commands.h>
#include <frc2/command/button/CommandXboxController.h>

namespace cmd {
    frc2::CommandPtr IntakeFullSequence();
    frc2::CommandPtr RemoveAlgaeLow(bool force=false);
    frc2::CommandPtr RemoveAlgaeHigh(bool force=false);
    frc2::CommandPtr IntakeFromSource();
    frc2::CommandPtr Outtake();

    frc2::CommandPtr SetElevatorPosition(units::meter_t height, bool force=false);
    frc2::CommandPtr SetL1(bool force=false);
    frc2::CommandPtr SetL2(bool force=false);
    frc2::CommandPtr SetL3(bool force=false);
    frc2::CommandPtr SetL4(bool force=false);
    frc2::CommandPtr SetClimb(bool force=false);
    frc2::CommandPtr SetClearAlgaeLow(bool force=false);
    frc2::CommandPtr SetClearAlgaeHigh(bool force=false);
}