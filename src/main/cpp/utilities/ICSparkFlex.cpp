#include "utilities/ICSparkFlex.h"

ICSparkFlex::ICSparkFlex(int deviceID, units::ampere_t currentLimit)
    : SparkFlex(deviceID, rev::spark::SparkLowLevel::MotorType::kBrushless),
      ICSpark(this, GetEncoder(), configAccessor, currentLimit) {}

void ICSparkFlex::Set(double speed) { ICSpark::SetDutyCycle(speed); }

void ICSparkFlex::SetVoltage(units::volt_t output) {
  ICSpark::SetVoltage(output);
}

double ICSparkFlex::Get() const { return ICSpark::GetDutyCycle(); }

void ICSparkFlex::StopMotor() { ICSpark::StopMotor(); }

void ICSparkFlex::UseExternalEncoder(int countsPerRev) {
  ICSpark::UseRelativeEncoder(SparkFlex::GetExternalEncoder(), countsPerRev);
}