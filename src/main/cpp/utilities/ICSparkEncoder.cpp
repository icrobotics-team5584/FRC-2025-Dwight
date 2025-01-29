#include <utilities/ICSparkEncoder.h>
#include <frc/RobotBase.h>
#include <utility>
#include <frc/smartdashboard/SmartDashboard.h>

ICSparkEncoder::ICSparkEncoder(rev::spark::SparkRelativeEncoder& inbuilt) : _inbuilt(inbuilt) {}

double ICSparkEncoder::GetPosition() {
  switch (_selected) {
    case ABSOLUTE:
      return frc::RobotBase::IsSimulation() ? _absoluteSimPos : _absolute->GetPosition();
    case RELATIVE:
      return _relative->GetPosition();
    case INBUILT:
    default:
      return _inbuilt.GetPosition();
  }
}

double ICSparkEncoder::GetVelocity() {
  switch (_selected) {
    case ABSOLUTE:
      return _absolute->GetVelocity();
    case RELATIVE:
      return _relative->GetVelocity();
    case INBUILT:
    default:
      return _inbuilt.GetVelocity();
  }
}

void ICSparkEncoder::SetPosition(double pos) {
  if(_relative){
    _relative->SetPosition(pos);
  }
  _inbuilt.SetPosition(pos);
  _absoluteSimPos = pos; // Doesn't do anything for a real life absolute encoder
}

void ICSparkEncoder::UseAbsolute(rev::spark::SparkAbsoluteEncoder& encoder) {
  _selected = ABSOLUTE;
  _absolute = std::make_unique<rev::spark::SparkAbsoluteEncoder>(encoder);
}

void ICSparkEncoder::UseRelative(rev::spark::SparkFlexExternalEncoder& encoder) {
  _selected = RELATIVE;
  _relative = std::make_unique<rev::spark::SparkFlexExternalEncoder>(encoder);
}

void ICSparkEncoder::UseRelative(rev::spark::SparkMaxAlternateEncoder& encoder) {
  _selected = RELATIVE;
  _relative = std::make_unique<rev::spark::SparkMaxAlternateEncoder>(encoder);
}
