#include "utilities/ICSparkConfig.h"
#include <tuple>

template <typename T>
void MergeOptional(std::optional<T>& target, const std::optional<T>& source) {
    if (source) {
        target = source;
    }
}

void ICSparkConfig::FillREVConfig(rev::spark::SparkBaseConfig& config) const {
  if (inverted)
    config.Inverted(*inverted);
  if (idleMode)
    config.SetIdleMode(*idleMode);
  if (smartCurrentStallLimit && smartCurrentfreeLimit && smartCurrentVelocityLimit)
    config.SmartCurrentLimit(smartCurrentStallLimit->value(), smartCurrentfreeLimit->value(),
                             smartCurrentVelocityLimit->value());
  if (secondaryCurrentLimit && secondaryCurrentLimitChopCycles)
    config.SecondaryCurrentLimit(secondaryCurrentLimit->value(), *secondaryCurrentLimitChopCycles);
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
  if (signals.busVoltagePeriodMs)
    config.signals.BusVoltagePeriodMs(signals.busVoltagePeriodMs->value());
  if (signals.outputCurrentPeriodMs)
    config.signals.OutputCurrentPeriodMs(signals.outputCurrentPeriodMs->value());
  if (signals.motorTemperaturePeriodMs)
    config.signals.MotorTemperaturePeriodMs(signals.motorTemperaturePeriodMs->value());
  if (signals.limitsPeriodMs)
    config.signals.LimitsPeriodMs(signals.limitsPeriodMs->value());
  if (signals.faultsPeriodMs)
    config.signals.FaultsPeriodMs(signals.faultsPeriodMs->value());
  if (signals.faultsAlwaysOn)
    config.signals.FaultsAlwaysOn(*signals.faultsAlwaysOn);
  if (signals.warningsPeriodMs)
    config.signals.WarningsPeriodMs(signals.warningsPeriodMs->value());
  if (signals.warningsAlwaysOn)
    config.signals.WarningsAlwaysOn(*signals.warningsAlwaysOn);
  if (signals.primaryEncoderVelocityPeriodMs)
    config.signals.PrimaryEncoderVelocityPeriodMs(signals.primaryEncoderVelocityPeriodMs->value());
  if (signals.primaryEncoderVelocityAlwaysOn)
    config.signals.PrimaryEncoderVelocityAlwaysOn(*signals.primaryEncoderVelocityAlwaysOn);
  if (signals.primaryEncoderPositionPeriodMs)
    config.signals.PrimaryEncoderPositionPeriodMs(signals.primaryEncoderPositionPeriodMs->value());
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

void ICSparkConfig::Adjust(const ICSparkConfig& other) {
    // Top level configs
    MergeOptional(inverted, other.inverted);
    MergeOptional(idleMode, other.idleMode);
    MergeOptional(smartCurrentStallLimit, other.smartCurrentStallLimit);
    MergeOptional(smartCurrentfreeLimit, other.smartCurrentfreeLimit);
    MergeOptional(smartCurrentVelocityLimit, other.smartCurrentVelocityLimit);
    MergeOptional(secondaryCurrentLimit, other.secondaryCurrentLimit);
    MergeOptional(secondaryCurrentLimitChopCycles, other.secondaryCurrentLimitChopCycles);
    MergeOptional(openLoopRampRate, other.openLoopRampRate);
    MergeOptional(closedLoopRampRate, other.closedLoopRampRate);
    MergeOptional(voltageCompensationNominalVoltage, other.voltageCompensationNominalVoltage);
    MergeOptional(followCanId, other.followCanId);
    MergeOptional(followInverted, other.followInverted);

    // Absolute Encoder
    MergeOptional(absoluteEncoder.inverted, other.absoluteEncoder.inverted);
    MergeOptional(absoluteEncoder.positionConversionFactor, other.absoluteEncoder.positionConversionFactor);
    MergeOptional(absoluteEncoder.velocityConversionFactor, other.absoluteEncoder.velocityConversionFactor);
    MergeOptional(absoluteEncoder.zeroOffset, other.absoluteEncoder.zeroOffset);
    MergeOptional(absoluteEncoder.averageDepth, other.absoluteEncoder.averageDepth);
    MergeOptional(absoluteEncoder.startPulseUs, other.absoluteEncoder.startPulseUs);
    MergeOptional(absoluteEncoder.endPulseUs, other.absoluteEncoder.endPulseUs);
    MergeOptional(absoluteEncoder.zeroCentered, other.absoluteEncoder.zeroCentered);

    // Analog sensor
    MergeOptional(analogSensor.inverted, other.analogSensor.inverted);
    MergeOptional(analogSensor.positionConversionFactor, other.analogSensor.positionConversionFactor);
    MergeOptional(analogSensor.velocityConversionFactor, other.analogSensor.velocityConversionFactor);

    // Closed loop global settings
    MergeOptional(closedLoop.positionWrappingEnabled, other.closedLoop.positionWrappingEnabled);
    MergeOptional(closedLoop.positionWrappingMinInput, other.closedLoop.positionWrappingMinInput);
    MergeOptional(closedLoop.positionWrappingMaxInput, other.closedLoop.positionWrappingMaxInput);
    MergeOptional(closedLoop.feedbackSensor, other.closedLoop.feedbackSensor);

    // Helper lambda to merge a closed-loop slot
    auto MergeSlot = [](auto &targetSlot, const auto &sourceSlot) {
        MergeOptional(targetSlot.p, sourceSlot.p);
        MergeOptional(targetSlot.i, sourceSlot.i);
        MergeOptional(targetSlot.d, sourceSlot.d);
        MergeOptional(targetSlot.velocityFF, sourceSlot.velocityFF);
        MergeOptional(targetSlot.dFilter, sourceSlot.dFilter);
        MergeOptional(targetSlot.iZone, sourceSlot.iZone);
        MergeOptional(targetSlot.minOutput, sourceSlot.minOutput);
        MergeOptional(targetSlot.maxOutput, sourceSlot.maxOutput);
        MergeOptional(targetSlot.iMaxAccum, sourceSlot.iMaxAccum);
    };

    MergeSlot(closedLoop.slots[0], other.closedLoop.slots[0]);
    MergeSlot(closedLoop.slots[1], other.closedLoop.slots[1]);
    MergeSlot(closedLoop.slots[2], other.closedLoop.slots[2]);
    MergeSlot(closedLoop.slots[3], other.closedLoop.slots[3]);

    // Encoder
    MergeOptional(encoder.inverted, other.encoder.inverted);
    MergeOptional(encoder.positionConversionFactor, other.encoder.positionConversionFactor);
    MergeOptional(encoder.velocityConversionFactor, other.encoder.velocityConversionFactor);
    MergeOptional(encoder.quadratureAverageDepth, other.encoder.quadratureAverageDepth);
    MergeOptional(encoder.quadratureMeasurementPeriod, other.encoder.quadratureMeasurementPeriod);
    MergeOptional(encoder.uvwAverageDepth, other.encoder.uvwAverageDepth);
    MergeOptional(encoder.uvwMeasurementPeriod, other.encoder.uvwMeasurementPeriod);

    // Limit switch
    MergeOptional(limitSwitch.forwardLimitSwitchEnabled, other.limitSwitch.forwardLimitSwitchEnabled);
    MergeOptional(limitSwitch.forwardLimitSwitchType, other.limitSwitch.forwardLimitSwitchType);
    MergeOptional(limitSwitch.reverseLimitSwitchEnabled, other.limitSwitch.reverseLimitSwitchEnabled);
    MergeOptional(limitSwitch.reverseLimitSwitchType, other.limitSwitch.reverseLimitSwitchType);

    // Signals
    MergeOptional(signals.appliedOutputPeriodMs, other.signals.appliedOutputPeriodMs);
    MergeOptional(signals.busVoltagePeriodMs, other.signals.busVoltagePeriodMs);
    MergeOptional(signals.outputCurrentPeriodMs, other.signals.outputCurrentPeriodMs);
    MergeOptional(signals.motorTemperaturePeriodMs, other.signals.motorTemperaturePeriodMs);
    MergeOptional(signals.limitsPeriodMs, other.signals.limitsPeriodMs);
    MergeOptional(signals.faultsPeriodMs, other.signals.faultsPeriodMs);
    MergeOptional(signals.faultsAlwaysOn, other.signals.faultsAlwaysOn);
    MergeOptional(signals.warningsPeriodMs, other.signals.warningsPeriodMs);
    MergeOptional(signals.warningsAlwaysOn, other.signals.warningsAlwaysOn);
    MergeOptional(signals.primaryEncoderVelocityPeriodMs, other.signals.primaryEncoderVelocityPeriodMs);
    MergeOptional(signals.primaryEncoderVelocityAlwaysOn, other.signals.primaryEncoderVelocityAlwaysOn);
    MergeOptional(signals.primaryEncoderPositionPeriodMs, other.signals.primaryEncoderPositionPeriodMs);
    MergeOptional(signals.primaryEncoderPositionAlwaysOn, other.signals.primaryEncoderPositionAlwaysOn);
    MergeOptional(signals.analogVoltagePeriodMs, other.signals.analogVoltagePeriodMs);
    MergeOptional(signals.analogVoltageAlwaysOn, other.signals.analogVoltageAlwaysOn);
    MergeOptional(signals.analogVelocityPeriodMs, other.signals.analogVelocityPeriodMs);
    MergeOptional(signals.analogVelocityAlwaysOn, other.signals.analogVelocityAlwaysOn);
    MergeOptional(signals.analogPositionPeriodMs, other.signals.analogPositionPeriodMs);
    MergeOptional(signals.analogPositionAlwaysOn, other.signals.analogPositionAlwaysOn);
    MergeOptional(signals.externalOrAltEncoderVelocity, other.signals.externalOrAltEncoderVelocity);
    MergeOptional(signals.externalOrAltEncoderVelocityAlwaysOn, other.signals.externalOrAltEncoderVelocityAlwaysOn);
    MergeOptional(signals.externalOrAltEncoderPosition, other.signals.externalOrAltEncoderPosition);
    MergeOptional(signals.externalOrAltEncoderPositionAlwaysOn, other.signals.externalOrAltEncoderPositionAlwaysOn);
    MergeOptional(signals.absoluteEncoderVelocityPeriodMs, other.signals.absoluteEncoderVelocityPeriodMs);
    MergeOptional(signals.absoluteEncoderVelocityAlwaysOn, other.signals.absoluteEncoderVelocityAlwaysOn);
    MergeOptional(signals.absoluteEncoderPositionPeriodMs, other.signals.absoluteEncoderPositionPeriodMs);
    MergeOptional(signals.absoluteEncoderPositionAlwaysOn, other.signals.absoluteEncoderPositionAlwaysOn);
    MergeOptional(signals.iAccumulationPeriodMs, other.signals.iAccumulationPeriodMs);
    MergeOptional(signals.iAccumulationAlwaysOn, other.signals.iAccumulationAlwaysOn);

    // Soft limit
    MergeOptional(softLimit.forwardSoftLimit, other.softLimit.forwardSoftLimit);
    MergeOptional(softLimit.forwardSoftLimitEnabled, other.softLimit.forwardSoftLimitEnabled);
    MergeOptional(softLimit.reverseSoftLimit, other.softLimit.reverseSoftLimit);
    MergeOptional(softLimit.reverseSoftLimitEnabled, other.softLimit.reverseSoftLimitEnabled);
}