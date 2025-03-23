#include "utilities/ICSparkFlex.h"

ICSparkFlex::ICSparkFlex(int deviceID)
    : SparkFlex(deviceID, rev::spark::SparkLowLevel::MotorType::kBrushless),
      ICSpark(this, GetEncoder(), configAccessor) {}

void ICSparkFlex::Set(double speed) { ICSpark::SetDutyCycle(speed); }

void ICSparkFlex::SetVoltage(units::volt_t output) {
  ICSpark::SetVoltage(output);
}

double ICSparkFlex::Get() const { return ICSpark::GetDutyCycle(); }

void ICSparkFlex::StopMotor() { ICSpark::StopMotor(); }

void ICSparkFlex::UseExternalEncoder() {
  ICSpark::UseRelativeEncoder(SparkFlex::GetExternalEncoder());
}