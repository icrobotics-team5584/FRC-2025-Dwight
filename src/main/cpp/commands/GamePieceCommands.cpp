#include "subsystems/SubIntake.h"
#include "subsystems/SubEndEffector.h"
#include "subsystems/SubElevator.h"
#include "commands/GamePieceCommands.h"
#include "subsystems/SubFunnel.h"

namespace cmd {
    frc2::CommandPtr IntakeFullSequence() {
        return SubIntake::GetInstance().Deploy().AndThen(
            SubIntake::GetInstance().Intake().AlongWith(
                SubEndEffector::GetInstance().FeedUp()
                ).Until( [] {return SubEndEffector::GetInstance().CheckLineBreakHigher();})
                .AndThen(SubEndEffector::GetInstance().StopMotor().AndThen(SubIntake::GetInstance().Stow()))
        );
    }

    frc2::CommandPtr RemoveAlgae() {
        return SubEndEffector::GetInstance().FeedDown().AlongWith(SubElevator::GetInstance().ManualElevatorMovementAlgae());
    }

    //frc2::CommandPtr IntakeFromSource() {
   // return SubFunnel::GetInstance().FeedDownFunnel().AlongWith(SubEndEffector::GetInstance().FeedDown()).Until([this] {return SubEndEffector::GetInstance().CheckLineBreakHigher();})
    //.AndThen(SubEndEffector::GetInstance().FeedDownSLOW().Until([this] {return SubEndEffector::GetInstance().CheckLineBreakLower();}));
    //}

    frc2::CommandPtr IntakeFromSource() {
        return SubFunnel::GetInstance().FeedDownFunnel().AlongWith(SubEndEffector::GetInstance().FeedDown()).Until([] {return SubEndEffector::GetInstance().CheckLineBreakHigher();})
        .AndThen(SubEndEffector::GetInstance().FeedDownSLOW().Until([] {return SubEndEffector::GetInstance().CheckLineBreakLower();}));
    }



}