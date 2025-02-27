#include "subsystems/SubEndEffector.h"
#include "subsystems/SubElevator.h"
#include "subsystems/SubClimber.h"
#include "commands/GamePieceCommands.h"
#include "subsystems/SubFunnel.h"
#include <frc2/command/Commands.h>


namespace cmd {

frc2::CommandPtr ClimbUpSequence() {
    return SubElevator::GetInstance().CmdSetClimb()
            .AndThen(frc2::cmd::WaitUntil([]{return SubElevator::GetInstance().IsAtTarget();}))
        .AndThen(SubClimber::GetInstance().ReadyClimber());
}

frc2::CommandPtr ClimbDownSequence() {
    return SubElevator::GetInstance().CmdSetClimb()
            .AndThen(frc2::cmd::WaitUntil([]{return SubElevator::GetInstance().IsAtTarget();}))
        .AndThen(SubClimber::GetInstance().Climb());
        //.AndThen(SubElevator::GetInstance().CmdElevatorToPosition(0_m)) // 0_m is the start height
        //.AndThen(SubClimber::GetInstance().StowClimber());
}

frc2::CommandPtr RemoveAlgaeLow(bool force) {
  return SubEndEffector::GetInstance().FeedDown().AlongWith(SetClearAlgaeLow(force));
}

frc2::CommandPtr RemoveAlgaeHigh(bool force) {
  return SubEndEffector::GetInstance().FeedDown().AlongWith(SetClearAlgaeHigh(force));
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

frc2::CommandPtr SetElevatorPosition(units::meter_t height, bool force) {
  return SubElevator::GetInstance().CmdElevatorToPosition(height).OnlyIf([force] {
    return (force || SubEndEffector::GetInstance().CheckLineBreakLower() ==
                     SubEndEffector::GetInstance().CheckLineBreakHigher());
  });
}

frc2::CommandPtr SetL1(bool force) { return cmd::SetElevatorPosition(SubElevator::_L1_HEIGHT, force); }
frc2::CommandPtr SetL2(bool force) { return cmd::SetElevatorPosition(SubElevator::_L2_HEIGHT, force); }
frc2::CommandPtr SetL3(bool force) { return cmd::SetElevatorPosition(SubElevator::_L3_HEIGHT, force); }
frc2::CommandPtr SetL4(bool force) { return cmd::SetElevatorPosition(SubElevator::_L4_HEIGHT, force); }

frc2::CommandPtr SetClimb(bool force) { return cmd::SetElevatorPosition(SubElevator::_CLIMB_HEIGHT, force); }

frc2::CommandPtr SetClearAlgaeLow(bool force) {
  return cmd::SetElevatorPosition(SubElevator::_ALGAE_LOW_HEIGHT, force);
}

frc2::CommandPtr SetClearAlgaeHigh(bool force) {
  return cmd::SetElevatorPosition(SubElevator::_ALGAE_HIGH_HEIGHT, force);
}
}  // namespace cmd //
