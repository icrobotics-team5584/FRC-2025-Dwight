#pragma once
#include <frc2/command/Commands.h>
#include <frc2/command/button/CommandXboxController.h>

namespace cmd {
    frc2::CommandPtr IntakeFullSequence();
    frc2::CommandPtr RemoveAlgaeLow(bool force=false);
    frc2::CommandPtr RemoveAlgaeHigh(bool force=false);
    frc2::CommandPtr ClimbUpSequence(bool force=false);
    frc2::CommandPtr ClimbDownSequence(bool force=false);
    frc2::CommandPtr IntakeFromSource();
    frc2::CommandPtr Outtake();

    frc2::CommandPtr SetElevatorPosition(units::meter_t height, bool force=false);
    frc2::CommandPtr SetElevatorL1(bool force=false);
    frc2::CommandPtr SetElevatorL2(bool force=false);
    frc2::CommandPtr SetElevatorL3(bool force=false);
    frc2::CommandPtr SetElevatorL4(bool force=false);
    frc2::CommandPtr SetElevatorClimb(bool force=false);
    frc2::CommandPtr SetClearAlgaeLow(bool force=false);
    frc2::CommandPtr SetClearAlgaeHigh(bool force=false);
    frc2::CommandPtr StowClimber();
}