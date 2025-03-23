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
  rev::spark::ClosedLoopSlot slotID = rev::spark::kSlot0;
  std::optional<double> p, i, d, velocityFF, dFilter, iZone;
  std::optional<double> minOutput, maxOutput;
  std::optional<double> iMaxAccum;
  struct {
    std::optional<units::turns_per_second_t> maxVelocity;
    std::optional<units::turns_per_second_squared_t> maxAcceleration;
    std::optional<units::turn_t> allowedClosedLoopError;
    std::optional<rev::spark::MAXMotionConfig::MAXMotionPositionMode> positionMode;
  } maxMotion;
};

struct ICSparkConfig {
  // Top level configs
  std::optional<bool> inverted;
  std::optional<rev::spark::SparkBaseConfig::IdleMode> idleMode;
  std::optional<units::ampere_t> smartCurrentStallLimit;
  std::optional<units::ampere_t> smartCurrentfreeLimit;
  std::optional<units::turns_per_second_t> smartCurrentVelocityLimit;
  std::optional<units::ampere_t> secondaryCurrentLimit;
  std::optional<int> secondaryCurrentLimitChopCycles = 0;
  std::optional<units::second_t> openLoopRampRate;
  std::optional<units::second_t> closedLoopRampRate;
  std::optional<units::volt_t> voltageCompensationNominalVoltage;
  std::optional<int> followCanId;
  std::optional<bool> followInverted = false;

  // Absolute Encoder
  struct {
    std::optional<bool> inverted;
    std::optional<double> positionConversionFactor;
    std::optional<double> velocityConversionFactor;
    std::optional<units::turn_t> zeroOffset;
    std::optional<int> averageDepth;
    std::optional<units::microsecond_t> startPulseUs;
    std::optional<units::microsecond_t> endPulseUs;
    std::optional<bool> zeroCentered;
  } absoluteEncoder;

  // Analog sensor
  struct {
    std::optional<bool> inverted;
    std::optional<double> positionConversionFactor;
    std::optional<double> velocityConversionFactor;
  } analogSensor;

  // Closed loop
  struct {
    std::optional<bool> positionWrappingEnabled;
    std::optional<units::turn_t> positionWrappingMinInput;
    std::optional<units::turn_t> positionWrappingMaxInput;
    std::optional<rev::spark::ClosedLoopConfig::FeedbackSensor> feedbackSensor;

    std::array<ClosedLoopSlotConfig, 3> slots{rev::spark::kSlot0, rev::spark::kSlot1,
                                              rev::spark::kSlot2, rev::spark::kSlot3};
  } closedLoop;

  // Encoder
  struct {
    std::optional<bool> inverted;
    std::optional<double> positionConversionFactor;
    std::optional<double> velocityConversionFactor;
    std::optional<int> quadratureAverageDepth;
    std::optional<units::millisecond_t> quadratureMeasurementPeriod;
    std::optional<int> uvwAverageDepth;
    std::optional<units::millisecond_t> uvwMeasurementPeriod;
  } encoder;

  // Limit switch
  struct {
    std::optional<bool> forwardLimitSwitchEnabled;
    std::optional<rev::spark::LimitSwitchConfig::Type> forwardLimitSwitchType;
    std::optional<bool> reverseLimitSwitchEnabled;
    std::optional<rev::spark::LimitSwitchConfig::Type> reverseLimitSwitchType;
  } limitSwitch;

  // Signals
  struct {
    std::optional<units::millisecond_t> appliedOutputPeriodMs;
    std::optional<bool> appliedOutputAlwaysOn;
    std::optional<units::millisecond_t> busVoltagePeriodMs;
    std::optional<bool> busVoltageAlwaysOn;
    std::optional<units::millisecond_t> outputCurrentPeriodMs;
    std::optional<bool> outputCurrentAlwaysOn;
    std::optional<units::millisecond_t> motorTemperaturePeriodMs;
    std::optional<bool> motorTemperatureAlwaysOn;
    std::optional<units::millisecond_t> limitsPeriodMs;
    std::optional<bool> limitsAlwaysOn;
    std::optional<units::millisecond_t> faultsPeriodMs;
    std::optional<bool> faultsAlwaysOn;
    std::optional<units::millisecond_t> warningsPeriodMs;
    std::optional<bool> warningsAlwaysOn;
    std::optional<units::millisecond_t> primaryEncoderVelocityPeriodMs;
    std::optional<bool> primaryEncoderVelocityAlwaysOn;
    std::optional<units::millisecond_t> primaryEncoderPositionPeriodMs;
    std::optional<bool> primaryEncoderPositionAlwaysOn;
    std::optional<units::millisecond_t> analogVoltagePeriodMs;
    std::optional<bool> analogVoltageAlwaysOn;
    std::optional<units::millisecond_t> analogVelocityPeriodMs;
    std::optional<bool> analogVelocityAlwaysOn;
    std::optional<units::millisecond_t> analogPositionPeriodMs;
    std::optional<bool> analogPositionAlwaysOn;
    std::optional<units::millisecond_t> externalOrAltEncoderVelocity;
    std::optional<bool> externalOrAltEncoderVelocityAlwaysOn;
    std::optional<units::millisecond_t> externalOrAltEncoderPosition;
    std::optional<bool> externalOrAltEncoderPositionAlwaysOn;
    std::optional<units::millisecond_t> absoluteEncoderVelocityPeriodMs;
    std::optional<bool> absoluteEncoderVelocityAlwaysOn;
    std::optional<units::millisecond_t> absoluteEncoderPositionPeriodMs;
    std::optional<bool> absoluteEncoderPositionAlwaysOn;
    std::optional<units::millisecond_t> iAccumulationPeriodMs;
    std::optional<bool> iAccumulationAlwaysOn;
  } signals;

  // Soft limit
  struct {
    std::optional<units::turn_t> forwardSoftLimit;
    std::optional<bool> forwardSoftLimitEnabled;
    std::optional<units::turn_t> reverseSoftLimit;
    std::optional<bool> reverseSoftLimitEnabled;
  } softLimit;

  void FillREVConfig(rev::spark::SparkBaseConfig& config) const {
    if (inverted)
      config.Inverted(*inverted);
    if (idleMode)
      config.SetIdleMode(*idleMode);
    if (smartCurrentStallLimit && smartCurrentfreeLimit && smartCurrentVelocityLimit)
      config.SmartCurrentLimit(smartCurrentStallLimit->value(), smartCurrentfreeLimit->value(),
                               smartCurrentVelocityLimit->value());
    if (secondaryCurrentLimit && secondaryCurrentLimitChopCycles)
      config.SecondaryCurrentLimit(secondaryCurrentLimit->value(),
                                   *secondaryCurrentLimitChopCycles);
    if (openLoopRampRate)
      config.OpenLoopRampRate(openLoopRampRate->value());
    if (closedLoopRampRate)
      config.ClosedLoopRampRate(closedLoopRampRate->value());
    if (voltageCompensationNominalVoltage)
      config.VoltageCompensation(voltageCompensationNominalVoltage->value());
    else
      config.DisableVoltageCompensation();
    if (followCanId && followInverted)
      config.Follow(*followCanId, *followInverted);

    // Absolute Encoder
    if (absoluteEncoder.inverted)
      config.absoluteEncoder.Inverted(*absoluteEncoder.inverted);
    if (absoluteEncoder.positionConversionFactor)
      config.absoluteEncoder.PositionConversionFactor(*absoluteEncoder.positionConversionFactor);
    if (absoluteEncoder.velocityConversionFactor)
      config.absoluteEncoder.VelocityConversionFactor(*absoluteEncoder.velocityConversionFactor);
    if (absoluteEncoder.zeroOffset)
      config.absoluteEncoder.ZeroOffset(absoluteEncoder.zeroOffset->value());
    if (absoluteEncoder.averageDepth)
      config.absoluteEncoder.AverageDepth(*absoluteEncoder.averageDepth);
    if (absoluteEncoder.startPulseUs)
      config.absoluteEncoder.StartPulseUs(absoluteEncoder.startPulseUs->value());
    if (absoluteEncoder.endPulseUs)
      config.absoluteEncoder.EndPulseUs(absoluteEncoder.endPulseUs->value());
    if (absoluteEncoder.zeroCentered)
      config.absoluteEncoder.ZeroCentered(*absoluteEncoder.zeroCentered);

    // Analog sensor
    if (analogSensor.inverted)
      config.analogSensor.Inverted(*analogSensor.inverted);
    if (analogSensor.positionConversionFactor)
      config.analogSensor.PositionConversionFactor(*analogSensor.positionConversionFactor);
    if (analogSensor.velocityConversionFactor)
      config.analogSensor.VelocityConversionFactor(*analogSensor.velocityConversionFactor);

    // Closed loop general
    if (closedLoop.positionWrappingEnabled)
      config.closedLoop.PositionWrappingEnabled(*closedLoop.positionWrappingEnabled);
    if (closedLoop.positionWrappingMinInput)
      config.closedLoop.PositionWrappingMinInput(closedLoop.positionWrappingMinInput->value());
    if (closedLoop.positionWrappingMaxInput)
      config.closedLoop.PositionWrappingMaxInput(closedLoop.positionWrappingMaxInput->value());
    if (closedLoop.feedbackSensor)
      config.closedLoop.SetFeedbackSensor(*closedLoop.feedbackSensor);

    // Closed loop - Slots
    for (auto& slot : closedLoop.slots) {
      if (slot.p)
        config.closedLoop.P(*slot.p, slot.slotID);
      if (slot.i)
        config.closedLoop.I(*slot.i, slot.slotID);
      if (slot.d)
        config.closedLoop.D(*slot.d, slot.slotID);
      if (slot.velocityFF)
        config.closedLoop.VelocityFF(*slot.velocityFF, slot.slotID);
      if (slot.dFilter)
        config.closedLoop.DFilter(*slot.dFilter, slot.slotID);
      if (slot.iZone)
        config.closedLoop.IZone(*slot.iZone, slot.slotID);
      if (slot.minOutput)
        config.closedLoop.MinOutput(*slot.minOutput, slot.slotID);
      if (slot.maxOutput)
        config.closedLoop.MaxOutput(*slot.maxOutput, slot.slotID);
      if (slot.iMaxAccum)
        config.closedLoop.IMaxAccum(*slot.iMaxAccum, slot.slotID);
      if (slot.maxMotion.maxVelocity)
        config.closedLoop.maxMotion.MaxVelocity(slot.maxMotion.maxVelocity->value(), slot.slotID);
      if (slot.maxMotion.maxAcceleration)
        config.closedLoop.maxMotion.MaxAcceleration(slot.maxMotion.maxAcceleration->value(),
                                                    slot.slotID);
      if (slot.maxMotion.allowedClosedLoopError)
        config.closedLoop.maxMotion.AllowedClosedLoopError(
            slot.maxMotion.allowedClosedLoopError->value(), slot.slotID);
      if (slot.maxMotion.positionMode)
        config.closedLoop.maxMotion.PositionMode(*slot.maxMotion.positionMode, slot.slotID);
    }

    // Encoder
    if (encoder.inverted)
      config.encoder.Inverted(*encoder.inverted);
    if (encoder.positionConversionFactor)
      config.encoder.PositionConversionFactor(*encoder.positionConversionFactor);
    if (encoder.velocityConversionFactor)
      config.encoder.VelocityConversionFactor(*encoder.velocityConversionFactor);
    if (encoder.quadratureAverageDepth)
      config.encoder.QuadratureAverageDepth(*encoder.quadratureAverageDepth);
    if (encoder.quadratureMeasurementPeriod)
      config.encoder.QuadratureMeasurementPeriod(encoder.quadratureMeasurementPeriod->value());
    if (encoder.uvwAverageDepth)
      config.encoder.UvwAverageDepth(*encoder.uvwAverageDepth);
    if (encoder.uvwMeasurementPeriod)
      config.encoder.UvwMeasurementPeriod(encoder.uvwMeasurementPeriod->value());

    // Limit switch
    if (limitSwitch.forwardLimitSwitchEnabled)
      config.limitSwitch.ForwardLimitSwitchEnabled(*limitSwitch.forwardLimitSwitchEnabled);
    if (limitSwitch.forwardLimitSwitchType)
      config.limitSwitch.ForwardLimitSwitchType(*limitSwitch.forwardLimitSwitchType);
    if (limitSwitch.reverseLimitSwitchEnabled)
      config.limitSwitch.ReverseLimitSwitchEnabled(*limitSwitch.reverseLimitSwitchEnabled);
    if (limitSwitch.reverseLimitSwitchType)
      config.limitSwitch.ReverseLimitSwitchType(*limitSwitch.reverseLimitSwitchType);

    // Signals
    if (signals.appliedOutputPeriodMs)
      config.signals.AppliedOutputPeriodMs(signals.appliedOutputPeriodMs->value());
    if (signals.appliedOutputAlwaysOn)
      config.signals.AppliedOutputAlwaysOn(*signals.appliedOutputAlwaysOn);
    if (signals.busVoltagePeriodMs)
      config.signals.BusVoltagePeriodMs(signals.busVoltagePeriodMs->value());
    if (signals.busVoltageAlwaysOn)
      config.signals.BusVoltageAlwaysOn(*signals.busVoltageAlwaysOn);
    if (signals.outputCurrentPeriodMs)
      config.signals.OutputCurrentPeriodMs(signals.outputCurrentPeriodMs->value());
    if (signals.outputCurrentAlwaysOn)
      config.signals.OutputCurrentAlwaysOn(*signals.outputCurrentAlwaysOn);
    if (signals.motorTemperaturePeriodMs)
      config.signals.MotorTemperaturePeriodMs(signals.motorTemperaturePeriodMs->value());
    if (signals.motorTemperatureAlwaysOn)
      config.signals.MotorTemperatureAlwaysOn(*signals.motorTemperatureAlwaysOn);
    if (signals.limitsPeriodMs)
      config.signals.LimitsPeriodMs(signals.limitsPeriodMs->value());
    if (signals.limitsAlwaysOn)
      config.signals.LimitsAlwaysOn(*signals.limitsAlwaysOn);
    if (signals.faultsPeriodMs)
      config.signals.FaultsPeriodMs(signals.faultsPeriodMs->value());
    if (signals.faultsAlwaysOn)
      config.signals.FaultsAlwaysOn(*signals.faultsAlwaysOn);
    if (signals.warningsPeriodMs)
      config.signals.WarningsPeriodMs(signals.warningsPeriodMs->value());
    if (signals.warningsAlwaysOn)
      config.signals.WarningsAlwaysOn(*signals.warningsAlwaysOn);
    if (signals.primaryEncoderVelocityPeriodMs)
      config.signals.PrimaryEncoderVelocityPeriodMs(
          signals.primaryEncoderVelocityPeriodMs->value());
    if (signals.primaryEncoderVelocityAlwaysOn)
      config.signals.PrimaryEncoderVelocityAlwaysOn(*signals.primaryEncoderVelocityAlwaysOn);
    if (signals.primaryEncoderPositionPeriodMs)
      config.signals.PrimaryEncoderPositionPeriodMs(
          signals.primaryEncoderPositionPeriodMs->value());
    if (signals.primaryEncoderPositionAlwaysOn)
      config.signals.PrimaryEncoderPositionAlwaysOn(*signals.primaryEncoderPositionAlwaysOn);
    if (signals.analogVoltagePeriodMs)
      config.signals.AnalogVoltagePeriodMs(signals.analogVoltagePeriodMs->value());
    if (signals.analogVoltageAlwaysOn)
      config.signals.AnalogVoltageAlwaysOn(*signals.analogVoltageAlwaysOn);
    if (signals.analogVelocityPeriodMs)
      config.signals.AnalogVelocityPeriodMs(signals.analogVelocityPeriodMs->value());
    if (signals.analogVelocityAlwaysOn)
      config.signals.AnalogVelocityAlwaysOn(*signals.analogVelocityAlwaysOn);
    if (signals.analogPositionPeriodMs)
      config.signals.AnalogPositionPeriodMs(signals.analogPositionPeriodMs->value());
    if (signals.analogPositionAlwaysOn)
      config.signals.AnalogPositionAlwaysOn(*signals.analogPositionAlwaysOn);
    if (signals.externalOrAltEncoderVelocity)
      config.signals.ExternalOrAltEncoderVelocity(signals.externalOrAltEncoderVelocity->value());
    if (signals.externalOrAltEncoderVelocityAlwaysOn)
      config.signals.ExternalOrAltEncoderVelocityAlwaysOn(
          *signals.externalOrAltEncoderVelocityAlwaysOn);
    if (signals.externalOrAltEncoderPosition)
      config.signals.ExternalOrAltEncoderPosition(signals.externalOrAltEncoderPosition->value());
    if (signals.externalOrAltEncoderPositionAlwaysOn)
      config.signals.ExternalOrAltEncoderPositionAlwaysOn(
          *signals.externalOrAltEncoderPositionAlwaysOn);
    if (signals.absoluteEncoderVelocityPeriodMs)
      config.signals.AbsoluteEncoderVelocityPeriodMs(
          signals.absoluteEncoderVelocityPeriodMs->value());
    if (signals.absoluteEncoderVelocityAlwaysOn)
      config.signals.AbsoluteEncoderVelocityAlwaysOn(*signals.absoluteEncoderVelocityAlwaysOn);
    if (signals.absoluteEncoderPositionPeriodMs)
      config.signals.AbsoluteEncoderPositionPeriodMs(
          signals.absoluteEncoderPositionPeriodMs->value());
    if (signals.absoluteEncoderPositionAlwaysOn)
      config.signals.AbsoluteEncoderPositionAlwaysOn(*signals.absoluteEncoderPositionAlwaysOn);
    if (signals.iAccumulationPeriodMs)
      config.signals.IAccumulationPeriodMs(signals.iAccumulationPeriodMs->value());
    if (signals.iAccumulationAlwaysOn)
      config.signals.IAccumulationAlwaysOn(*signals.iAccumulationAlwaysOn);

    // Soft limit
    if (softLimit.forwardSoftLimit)
      config.softLimit.ForwardSoftLimit(softLimit.forwardSoftLimit->value());
    if (softLimit.forwardSoftLimitEnabled)
      config.softLimit.ForwardSoftLimitEnabled(*softLimit.forwardSoftLimitEnabled);
    if (softLimit.reverseSoftLimit)
      config.softLimit.ReverseSoftLimit(softLimit.reverseSoftLimit->value());
    if (softLimit.reverseSoftLimitEnabled)
      config.softLimit.ReverseSoftLimitEnabled(*softLimit.reverseSoftLimitEnabled);
  }
};
