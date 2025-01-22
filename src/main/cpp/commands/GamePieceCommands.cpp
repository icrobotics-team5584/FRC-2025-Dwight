#include "subsystems/SubIntake.h"
#include "subsystems/SubEndEffector.h"
#include "subsystems/SubElevator.h"
#include <math.h>
#include <units/length.h>
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

    frc2::CommandPtr AutoAdjustElevatorHeight(int targetLevel) {
        return SubElevator::GetInstance().CmdElevatorToPosition(2_m*tan(55) + 1.8_m);
        /*
        this is a placeholder function - instructions for implementation below

        2_m will need to be replaced by the received distance from the laserCAN.

        leave tan(55) - it is a pre-determined constant. 55 is the complementary angle
        to the 35-degree angle of the end effector.

        1.8_m will need to be replaced by the respective constant for the targetLevel.
        the constant is the height of the elevator when it is directly next to the reef.
        it is probably best to do this using a map/array, with each level (L1, 2, 3, 4)
        mapped to its constant. currently, the constants (as per CAD) are as follows:
            L1 - 300mm  / 0.3m
            L2 - 500mm  / 0.5m
            L3 - 900mm  / 0.9m
            L4 - 1500mm / 1.5m
        these will need to be tuned when the elevator and end effector are actually ready.
        */
    frc2::CommandPtr RemoveAlgae() {
        return SubEndEffector::GetInstance().FeedDown().AlongWith(SubElevator::GetInstance().ManualElevatorMovementAlgae());
    }
}