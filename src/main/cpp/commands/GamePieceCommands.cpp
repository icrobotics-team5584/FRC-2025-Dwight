#include "subsystems/SubEndEffector.h"
#include "subsystems/SubElevator.h"
#include "commands/GamePieceCommands.h"
#include "subsystems/SubFunnel.h"
#include <frc2/command/Commands.h>


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
      .AndThen(frc2::cmd::WaitUntil([] { return SubElevator::GetInstance().IsAtTarget();}))
      .AndThen(SubFunnel::GetInstance().FeedDownFunnel())
      .AlongWith(SubEndEffector::GetInstance().FeedDown())
      .Until([] { return SubEndEffector::GetInstance().CheckLineBreakHigher(); })
      .AndThen(SubEndEffector::GetInstance().FeedDownSLOW().Until(
          [] { return SubEndEffector::GetInstance().CheckLineBreakLower(); }));
}

frc2::CommandPtr Outtake(){
  return SubEndEffector::GetInstance().FeedUpSLOW().AlongWith(SubFunnel::GetInstance().FeedUpFunnel());
}

frc2::CommandPtr SetL1() {
  return SubElevator::GetInstance().CmdSetL1().OnlyIf([] {
    return SubEndEffector::GetInstance().CheckLineBreakLower() ==
           SubEndEffector::GetInstance().CheckLineBreakHigher();
  });
}

frc2::CommandPtr SetL2(){
  return SubElevator::GetInstance().CmdSetL2().OnlyIf([] {
    return SubEndEffector::GetInstance().CheckLineBreakLower() ==
           SubEndEffector::GetInstance().CheckLineBreakHigher();
  });
}

frc2::CommandPtr SetL3(){
  return SubElevator::GetInstance().CmdSetL3().OnlyIf([] {
    return SubEndEffector::GetInstance().CheckLineBreakLower() ==
           SubEndEffector::GetInstance().CheckLineBreakHigher();
  });
}

frc2::CommandPtr SetL4(){
  return SubElevator::GetInstance().CmdSetL4().OnlyIf([] {
    return SubEndEffector::GetInstance().CheckLineBreakLower() ==
           SubEndEffector::GetInstance().CheckLineBreakHigher();
  });
}

}  // namespace cmd //