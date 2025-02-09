#include "subsystems/SubEndEffector.h"
#include "subsystems/SubElevator.h"
#include "commands/GamePieceCommands.h"
#include "subsystems/SubFunnel.h"

namespace cmd {

frc2::CommandPtr RemoveAlgae() {
  return SubEndEffector::GetInstance().FeedDown().AlongWith(
      SubElevator::GetInstance().ManualElevatorMovementAlgae());
}

frc2::CommandPtr IntakeFromSource() {
  return SubFunnel::GetInstance()
      .FeedDownFunnel()
      .AlongWith(SubEndEffector::GetInstance().FeedDown())
      .Until([] { return SubEndEffector::GetInstance().CheckLineBreakHigher(); })
      .AndThen(SubEndEffector::GetInstance().FeedDownSLOW().Until(
          [] { return SubEndEffector::GetInstance().CheckLineBreakLower(); }));
}

}  // namespace cmd