#include "subsystems/SubEndEffector.h"
#include "subsystems/SubElevator.h"
#include "subsystems/SubClimber.h"
#include "commands/GamePieceCommands.h"
#include "subsystems/SubFunnel.h"
#include <frc2/command/Commands.h>


namespace cmd {

frc2::CommandPtr ClimbUpSequence() {
  return SubElevator::GetInstance()
      .CmdSetLatch()
      .AndThen(frc2::cmd::WaitUntil([] { return SubElevator::GetInstance().IsAtTarget(); }))
      .AndThen(SubClimber::GetInstance().ReadyClimber().AlongWith(
          SubElevator::GetInstance().CmdSetClimb()));
}

frc2::CommandPtr ClimbDownSequence() {
    return SubElevator::GetInstance().CmdSetClimb()
            .AndThen(frc2::cmd::WaitUntil([]{return SubElevator::GetInstance().IsAtTarget();}))
        .AndThen(SubClimber::GetInstance().Climb());
        //.AndThen(SubElevator::GetInstance().CmdElevatorToPosition(0_m)) // 0_m is the start height
        //.AndThen(SubClimber::GetInstance().StowClimber());
}

frc2::CommandPtr RemoveAlgaeLow() {
  return SubEndEffector::GetInstance().RemoveAlgae().AlongWith(
      SubElevator::GetInstance().CmdSetClearLowAlgea());
}

frc2::CommandPtr RemoveAlgaeHigh() {
  return SubEndEffector::GetInstance().RemoveAlgae().AlongWith(
      SubElevator::GetInstance().CmdSetClearHighAlgea());
}

frc2::CommandPtr IntakeFromSource() {
  return SubElevator::GetInstance()
      .CmdSetSource()
      .AndThen(frc2::cmd::WaitUntil([] { return SubElevator::GetInstance().IsAtTarget();}))
      .AndThen(SubFunnel::GetInstance().FeedDownFunnel())
      .AlongWith(SubEndEffector::GetInstance().FeedDown())
      .Until([] { return SubEndEffector::GetInstance().CheckLineBreakHigher(); })
      .AndThen(SubEndEffector::GetInstance().FeedDownSLOW()
      .AlongWith(SubFunnel::GetInstance().FeedDownFunnelSLOW())
      .Until(
          [] { return SubEndEffector::GetInstance().CheckLineBreakLower(); }));
}

frc2::CommandPtr StowClimber() {
  return SubElevator::GetInstance()
      .CmdSetClimb()
      .AndThen(frc2::cmd::WaitUntil([] { return SubElevator::GetInstance().IsAtTarget(); }))
      .AndThen(SubClimber::GetInstance().StowClimber()) 
      .AndThen(frc2::cmd::WaitUntil([] { return SubClimber::GetInstance().IsAtTarget(); }))
      .AndThen(SubElevator::GetInstance().CmdSetSource());
}

frc2::CommandPtr Outtake(){
  return SubEndEffector::GetInstance().FeedUpSLOW().AlongWith(SubFunnel::GetInstance().FeedUpFunnel());
}
}  // namespace cmd //
