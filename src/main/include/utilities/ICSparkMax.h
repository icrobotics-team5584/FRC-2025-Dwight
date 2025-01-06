#pragma once

#include <rev/SparkMax.h>

#include "utilities/ICSpark.h"

/**
 * Helper class to setup an ICSpark to control a Spark Max. See ICSpark for
 * further details.
 */
class ICSparkMax : public rev::spark::SparkMax, public ICSpark {
 public:
  ICSparkMax(int deviceID, units::ampere_t currentLimit);
  static constexpr int NEO_ENCODER_RESOLUTION = 42;

  /**
   * Stop the motor until Set is called again or closed loop control is started.
   */
  void StopMotor() override;

  /**
   * Sets the duty cycle of a speed controller.
   *
   * @param speed The duty cycle to set. Value should be between -1.0 and 1.0.
   */
  void Set(double speed) override;

  /**
   * Sets the voltage of a speed controller.
   *
   * @param output The voltage to set.
   */
  void SetVoltage(units::volt_t output) override;

  /**
   * Common interface for getting the current set speed of a speed controller.
   *
   * @return The current set speed.  Value is between -1.0 and 1.0.
   */
  double Get() const override;

  /**
   * Use an alternate quadrature (relative) encoder connected to the Spark Max's
   * data port as the feedback device. To use an absolute encoder, see
   * UseAbsoluteEncoder() in ICSpark. This will be used for dashboard displays and
   * PID feedback. The pins of the data port are defined as:
   *
   * Pin 4 (Forward Limit Switch): Index
   * Pin 6 (Multi-function): Encoder A
   * Pin 8 (Reverse Limit Switch): Encoder B
   *
   * This call will disable support for the limit switch inputs.
   *
   * @param countsPerRev Number of encoder counts per revolution of the sensor.
   * Defaulted to 8192 which is the value for a REV throughbore encoder.
   */
  void UseAlternateEncoder(int countsPerRev = 8192);
};