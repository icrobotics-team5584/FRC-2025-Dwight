#pragma once

#include <units/current.h>
#include <units/voltage.h>
#include <units/time.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <optional>
#include <array>
#include <rev/config/SparkBaseConfig.h>

struct ClosedLoopSlotConfig {
  rev::spark::ClosedLoopSlot slotID;
  std::optional<double> p = std::nullopt; 
  std::optional<double> i = std::nullopt;
  std::optional<double> d = std::nullopt;
  std::optional<double> velocityFF = std::nullopt;
  std::optional<double> dFilter = std::nullopt;
  std::optional<double> iZone = std::nullopt;
  std::optional<double> iMaxAccum = std::nullopt;
  std::optional<double> minOutput = std::nullopt;
  std::optional<double> maxOutput = std::nullopt;
  struct {
    std::optional<units::turns_per_second_t> maxVelocity = std::nullopt;
    std::optional<units::turns_per_second_squared_t> maxAcceleration = std::nullopt;
    std::optional<units::turn_t> allowedClosedLoopError = std::nullopt;
    std::optional<rev::spark::MAXMotionConfig::MAXMotionPositionMode> positionMode = std::nullopt;
  } maxMotion = {};
};

struct ICSparkConfig {
  // Top level configs
  std::optional<bool> inverted;
  std::optional<rev::spark::SparkBaseConfig::IdleMode> idleMode = std::nullopt;
  std::optional<units::ampere_t> smartCurrentStallLimit = 40_A; // Safe-ish default
  std::optional<units::ampere_t> smartCurrentfreeLimit = 0_A;
  std::optional<units::turns_per_second_t> smartCurrentVelocityLimit = 20000_rpm;
  std::optional<units::ampere_t> secondaryCurrentLimit = std::nullopt;
  std::optional<int> secondaryCurrentLimitChopCycles = 0;
  std::optional<units::second_t> openLoopRampRate = std::nullopt;
  std::optional<units::second_t> closedLoopRampRate = std::nullopt;
  std::optional<units::volt_t> voltageCompensationNominalVoltage = std::nullopt;
  std::optional<int> followCanId = std::nullopt;
  std::optional<bool> followInverted = false;

  // Absolute Encoder
  struct {
    std::optional<bool> inverted = std::nullopt;
    std::optional<double> positionConversionFactor = std::nullopt;
    std::optional<double> velocityConversionFactor = std::nullopt;
    std::optional<units::turn_t> zeroOffset = std::nullopt;
    std::optional<int> averageDepth = std::nullopt;
    std::optional<units::microsecond_t> startPulseUs = std::nullopt;
    std::optional<units::microsecond_t> endPulseUs = std::nullopt;
    std::optional<bool> zeroCentered = std::nullopt;
  } absoluteEncoder;

  // Analog sensor
  struct {
    std::optional<bool> inverted = std::nullopt;
    std::optional<double> positionConversionFactor = std::nullopt;
    std::optional<double> velocityConversionFactor = std::nullopt;
  } analogSensor;

  // Closed loop
  struct {
    std::optional<bool> positionWrappingEnabled = std::nullopt;
    std::optional<units::turn_t> positionWrappingMinInput = std::nullopt;
    std::optional<units::turn_t> positionWrappingMaxInput = std::nullopt;
    std::optional<rev::spark::ClosedLoopConfig::FeedbackSensor> feedbackSensor = std::nullopt;

    std::array<ClosedLoopSlotConfig, 4> slots = {
        {{rev::spark::kSlot0}, {rev::spark::kSlot1}, {rev::spark::kSlot2}, {rev::spark::kSlot3}}};
  } closedLoop;

  // Encoder
  struct {
    std::optional<bool> inverted = std::nullopt;
    std::optional<double> positionConversionFactor = std::nullopt;
    std::optional<double> velocityConversionFactor = 1/60.0; // Transform RPM to RPS by default
    std::optional<int> quadratureAverageDepth = std::nullopt;
    std::optional<units::millisecond_t> quadratureMeasurementPeriod = std::nullopt;
    std::optional<int> uvwAverageDepth = std::nullopt;
    std::optional<units::millisecond_t> uvwMeasurementPeriod = std::nullopt;
  } encoder;

  // Limit switch
  struct {
    std::optional<bool> forwardLimitSwitchEnabled = std::nullopt;
    std::optional<rev::spark::LimitSwitchConfig::Type> forwardLimitSwitchType = std::nullopt;
    std::optional<bool> reverseLimitSwitchEnabled = std::nullopt;
    std::optional<rev::spark::LimitSwitchConfig::Type> reverseLimitSwitchType = std::nullopt;
  } limitSwitch;

  // Signals
  struct {
    std::optional<units::millisecond_t> appliedOutputPeriodMs = std::nullopt;
    std::optional<units::millisecond_t> busVoltagePeriodMs = std::nullopt;
    std::optional<units::millisecond_t> outputCurrentPeriodMs = std::nullopt;
    std::optional<units::millisecond_t> motorTemperaturePeriodMs = std::nullopt;
    std::optional<units::millisecond_t> limitsPeriodMs = std::nullopt;
    std::optional<units::millisecond_t> faultsPeriodMs = std::nullopt;
    std::optional<bool> faultsAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> warningsPeriodMs = std::nullopt;
    std::optional<bool> warningsAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> primaryEncoderVelocityPeriodMs = std::nullopt;
    std::optional<bool> primaryEncoderVelocityAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> primaryEncoderPositionPeriodMs = std::nullopt;
    std::optional<bool> primaryEncoderPositionAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> analogVoltagePeriodMs = std::nullopt;
    std::optional<bool> analogVoltageAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> analogVelocityPeriodMs = std::nullopt;
    std::optional<bool> analogVelocityAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> analogPositionPeriodMs = std::nullopt;
    std::optional<bool> analogPositionAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> externalOrAltEncoderVelocity = std::nullopt;
    std::optional<bool> externalOrAltEncoderVelocityAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> externalOrAltEncoderPosition = std::nullopt;
    std::optional<bool> externalOrAltEncoderPositionAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> absoluteEncoderVelocityPeriodMs = std::nullopt;
    std::optional<bool> absoluteEncoderVelocityAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> absoluteEncoderPositionPeriodMs = std::nullopt;
    std::optional<bool> absoluteEncoderPositionAlwaysOn = std::nullopt;
    std::optional<units::millisecond_t> iAccumulationPeriodMs = std::nullopt;
    std::optional<bool> iAccumulationAlwaysOn = std::nullopt;
  } signals;

  // Soft limit
  struct {
    std::optional<units::turn_t> forwardSoftLimit = std::nullopt;
    std::optional<bool> forwardSoftLimitEnabled = std::nullopt;
    std::optional<units::turn_t> reverseSoftLimit = std::nullopt;
    std::optional<bool> reverseSoftLimitEnabled = std::nullopt;
  } softLimit;

  /**
   * Apply this config to a REV SparkBaseConfig.
   */
  void FillREVConfig(rev::spark::SparkBaseConfig& config) const;

  /**
   * Edit the settings in this config to match the settings in a provided other
   * config.
   * Paramters that don't have a value in other will be ignored.
   * 
   * @param other The other config to pull settings from.
   */
  void Adjust(const ICSparkConfig& other);
};
