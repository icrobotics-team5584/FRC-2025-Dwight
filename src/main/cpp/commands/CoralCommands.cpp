#include "subsystems/SubIntake.h"
#include "subsystems/SubEndEffector.h"

namespace cmd {
    frc2::CommandPtr IntakeFullSequence() {
        return SubIntake::GetInstance().Deploy().AndThen(
            SubIntake::GetInstance().Intake().AlongWith(
                SubEndEffector::GetInstance().FeedUp()
                ).Until( [] {return SubEndEffector::GetInstance().CheckLineBreak();})
                .AndThen(SubEndEffector::GetInstance().StopMotor().AndThen(SubIntake::GetInstance().Stow()))
        );
    }
}