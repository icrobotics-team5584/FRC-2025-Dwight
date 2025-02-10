#include "subsystems/SubIntake.h"
#include "subsystems/SubEndEffector.h"
#include "subsystems/SubElevator.h"
#include "commands/GamePieceCommands.h"

namespace cmd {
    frc2::CommandPtr IntakeFullSequence() {
        return SubIntake::GetInstance().Deploy().AndThen(
            SubIntake::GetInstance().Intake().AlongWith(
                SubEndEffector::GetInstance().FeedUp()
                ).Until( [] {return SubEndEffector::GetInstance().CheckLineBreakHigher();})
                .AndThen(SubEndEffector::GetInstance().StopMotor().AndThen(SubIntake::GetInstance().Stow()))
        );
    }

    frc2::CommandPtr ClimbFullSequence() {
        return SubElevator::GetInstance().cmd
    }

    frc2::CommandPtr RemoveAlgae() {
        return SubEndEffector::GetInstance().FeedDown().AlongWith(SubElevator::GetInstance().ManualElevatorMovementAlgae());
    }
}