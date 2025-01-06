#include "utilities/ICSparkMax.h"

ICSparkMax::ICSparkMax(int deviceID, units::ampere_t currentLimit)
    : SparkMax(deviceID, rev::spark::SparkLowLevel::MotorType::kBrushless),
      ICSpark(this, GetEncoder(), SparkMax::configAccessor, currentLimit) {}

void ICSparkMax::Set(double speed) { ICSpark::SetDutyCycle(speed); }

void ICSparkMax::SetVoltage(units::volt_t output) {
  ICSpark::SetVoltage(output);
}

double ICSparkMax::Get() const { return ICSpark::GetDutyCycle(); }

void ICSparkMax::StopMotor() { ICSpark::StopMotor(); }

void ICSparkMax::UseAlternateEncoder(int countsPerRev) {
  ICSpark::UseRelativeEncoder(SparkMax::GetAlternateEncoder(), countsPerRev);
}