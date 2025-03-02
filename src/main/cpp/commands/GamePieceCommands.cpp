#include "subsystems/SubEndEffector.h"
#include "subsystems/SubElevator.h"
#include "subsystems/SubClimber.h"
#include "commands/GamePieceCommands.h"
#include "subsystems/SubFunnel.h"
#include <frc2/command/Commands.h>

namespace cmd {

frc2::CommandPtr ClimbUpSequence(bool force) {
  return SubElevator::GetInstance()
      .CmdSetLatch()
      .AndThen(frc2::cmd::WaitUntil([] {
    return SubElevator::GetInstance().IsAtTarget(); }))
      .AndThen(SubClimber::GetInstance().ReadyClimber().AlongWith(SetElevatorClimb(force)))
      .OnlyIf([force] {
    return (force || (SubEndEffector::GetInstance().IsCoralSecure() && !SubClimber::GetInstance().IsOut()));});
}

frc2::CommandPtr ClimbDownSequence(bool force) {
  return SetElevatorClimb(force)
      .AndThen(frc2::cmd::WaitUntil([] { return SubElevator::GetInstance().IsAtTarget(); }))
      .AndThen(SubClimber::GetInstance().Climb())
      .OnlyIf([force] {
    return (force || (SubEndEffector::GetInstance().IsCoralSecure() && !SubClimber::GetInstance().IsOut()));});
}

frc2::CommandPtr RemoveAlgaeLow(bool force) {
  return SubEndEffector::GetInstance().RemoveAlgae().AlongWith(SetClearAlgaeLow(force));
}

frc2::CommandPtr RemoveAlgaeHigh(bool force) {
  return SubEndEffector::GetInstance().RemoveAlgae().AlongWith(SetClearAlgaeHigh(force));
}

frc2::CommandPtr IntakeFromSource() {
  return SubElevator::GetInstance()
      .CmdSetSource()
      .AndThen(frc2::cmd::WaitUntil([] { return SubElevator::GetInstance().IsAtTarget(); }))
      .AndThen(SubFunnel::GetInstance().FeedDownFunnel())
      .AlongWith(SubEndEffector::GetInstance().FeedDown())
      .Until([] { return SubEndEffector::GetInstance().CheckLineBreakHigher(); })
      .AndThen(SubEndEffector::GetInstance().FeedDownSLOW()
      .AlongWith(SubFunnel::GetInstance().FeedDownFunnelSLOW())
      .Until(
          [] { return SubEndEffector::GetInstance().CheckLineBreakLower(); }));
}

frc2::CommandPtr SetElevatorPosition(units::meter_t height, bool force) {
  return SubElevator::GetInstance().CmdElevatorToPosition(height).OnlyIf([force] {
    return (force || (SubEndEffector::GetInstance().IsCoralSecure() && !SubClimber::GetInstance().IsOut()));
  });
}

frc2::CommandPtr SetElevatorL1(bool force) { return cmd::SetElevatorPosition(SubElevator::_L1_HEIGHT, force); }
frc2::CommandPtr SetElevatorL2(bool force) { return cmd::SetElevatorPosition(SubElevator::_L2_HEIGHT, force); }
frc2::CommandPtr SetElevatorL3(bool force) { return cmd::SetElevatorPosition(SubElevator::_L3_HEIGHT, force); }
frc2::CommandPtr SetElevatorL4(bool force) { return cmd::SetElevatorPosition(SubElevator::_L4_HEIGHT, force); }

frc2::CommandPtr SetElevatorClimb(bool force) { return cmd::SetElevatorPosition(SubElevator::_CLIMB_HEIGHT, force); }

frc2::CommandPtr SetClearAlgaeLow(bool force) {
  return cmd::SetElevatorPosition(SubElevator::_ALGAE_LOW_HEIGHT, force);
}

frc2::CommandPtr SetClearAlgaeHigh(bool force) {
  return cmd::SetElevatorPosition(SubElevator::_ALGAE_HIGH_HEIGHT, force);
}
  // namespace cmd
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
