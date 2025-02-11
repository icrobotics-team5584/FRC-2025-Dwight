#include "subsystems/SubEndEffector.h"
#include "subsystems/SubElevator.h"
#include "commands/GamePieceCommands.h"
#include "subsystems/SubFunnel.h"

namespace cmd {

frc2::CommandPtr RemoveAlgaeLow() {
  return SubEndEffector::GetInstance().FeedDown().AlongWith(
      SubElevator::GetInstance().CmdSetClearLowAlgea());
}

frc2::CommandPtr RemoveAlgaeHigh() {
  return SubEndEffector::GetInstance().FeedDown().AlongWith(
      SubElevator::GetInstance().CmdSetClearHighAlgea());
}

frc2::CommandPtr IntakeFromSource() {
  return SubElevator::GetInstance()
      .CmdSetSource()
      .Until([] { return SubElevator::GetInstance().IsAtTarget();})
      .AndThen(SubFunnel::GetInstance().FeedDownFunnel())
      .AlongWith(SubEndEffector::GetInstance().FeedDown())
      .Until([] { return SubEndEffector::GetInstance().CheckLineBreakHigher(); })
      .AndThen(SubEndEffector::GetInstance().FeedDownSLOW().Until(
          [] { return SubEndEffector::GetInstance().CheckLineBreakLower(); }))
          .AndThen([] {SubEndEffector::GetInstance().HasCoral = true;});
}

}  // namespace cmd //