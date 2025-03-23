#include "utilities/ICSpark.h"

#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/MathExtras.h>
#include <units/voltage.h>
#include <cstdlib>
#include <iostream>
#include <rev/ClosedLoopSlot.h>

ICSpark::ICSpark(rev::spark::SparkBase* spark, rev::spark::SparkRelativeEncoder& inbuiltEncoder,
                 rev::spark::SparkBaseConfigAccessor& configAccessor)
    : _spark(spark),
      _sparkConfigAccessor(configAccessor),
      _encoder(inbuiltEncoder),
      _simSpark(spark, &_simMotor) {
  // Apply the IC default configuration.
  // Includes setting vel conversion factor to use revs per sec not revs per min and a safeish
  // current limit.
  ICSparkConfig defaultConfig;
  AdjustConfig(defaultConfig);
}

void ICSpark::InitSendable(wpi::SendableBuilder& builder) {
  auto GetMaxMotionVel = [&] {
    return _configCache.closedLoop.slots[0].maxMotion.maxVelocity.value_or(0_tps);
  };
  auto GetMaxMotionAccel = [&] {
    return _configCache.closedLoop.slots[0].maxMotion.maxAcceleration.value_or(0_tr_per_s_sq);
  };
  // clang-format off
  //----------------------- Label ------------------------ Getter ------------------------------------------------ Setter -------------------------------------------------
  builder.AddDoubleProperty("Position",                   [&] { return GetPosition().value(); },                  nullptr);
  builder.AddDoubleProperty("Velocity",                   [&] { return GetVelocity().value(); },                  nullptr);
  builder.AddDoubleProperty("Voltage",                    [&] { return GetMotorVoltage().value(); },              nullptr);
  builder.AddDoubleProperty("Position Target",            [&] { return _positionTarget.value(); },                [&](double targ) { SetPositionTarget(targ*1_tr); });
  builder.AddDoubleProperty("Velocity Target",            [&] { return _velocityTarget.value(); },                [&](double targ) { SetVelocityTarget(targ*1_tps); });
  builder.AddDoubleProperty("Profile Position Target",    [&] { return _latestMotionTarget.position.value(); },   [&](double targ) { SetMotionProfileTarget(targ*1_tr); });
  builder.AddDoubleProperty("Profile Velocity Target",    [&] { return _latestMotionTarget.velocity.value(); },   nullptr);
  builder.AddDoubleProperty("Gains/FB P Gain",            [&] { return _rioPidController.GetP(); },               [&](double P) { SetFeedbackProportional(P); });
  builder.AddDoubleProperty("Gains/FB I Gain",            [&] { return _rioPidController.GetI(); },               [&](double I) { SetFeedbackIntegral(I); });
  builder.AddDoubleProperty("Gains/FB D Gain",            [&] { return _rioPidController.GetD(); },               [&](double D) { SetFeedbackDerivative(D); });
  builder.AddDoubleProperty("Gains/FF S Gain",            [&] { return _feedforwardStaticFriction.value(); },     [&](double S) { SetFeedforwardStaticFriction(S*1_V); });
  builder.AddDoubleProperty("Gains/FF V Gain",            [&] { return _feedforwardVelocity.value(); },           [&](double V) { SetFeedforwardVelocity(VoltsPerTps{V}); });
  builder.AddDoubleProperty("Gains/FF A Gain",            [&] { return _feedforwardAcceleration.value(); },       [&](double A) { SetFeedforwardAcceleration(VoltsPerTpsSq{A}); });
  builder.AddDoubleProperty("Gains/FF Linear G Gain",     [&] { return _feedforwardLinearGravity.value(); },      [&](double lG) { SetFeedforwardLinearGravity(lG*1_V); });
  builder.AddDoubleProperty("Gains/FF Rotational G Gain", [&] { return _feedforwardRotationalGravity.value(); },  [&](double rG) { SetFeedforwardRotationalGravity(rG*1_V); });
  builder.AddDoubleProperty("Motion Config/Max vel",      [&] { return GetMaxMotionVel().value(); },              [&](double vel) { SetMotionMaxVel(vel*1_tps); });
  builder.AddDoubleProperty("Motion Config/Max accel",    [&] { return GetMaxMotionAccel().value(); },            [&](double accel) { SetMotionMaxAccel(accel*1_tr_per_s_sq); });
  // clang-format on
}

rev::REVLibError ICSpark::Configure(ICSparkConfig& config,
                                    rev::spark::SparkBase::ResetMode resetMode,
                                    rev::spark::SparkBase::PersistMode persistMode) {
  auto& slot0 = config.closedLoop.slots[0];
  auto& oldSlot0 = _configCache.closedLoop.slots[0];

  // Motion profile config for sim and feed forward model
  _motionProfile = frc::TrapezoidProfile<units::turns>{{
      slot0.maxMotion.maxVelocity.value_or(oldSlot0.maxMotion.maxVelocity.value_or(0_tps)),
      slot0.maxMotion.maxAcceleration.value_or(oldSlot0.maxMotion.maxAcceleration.value_or(0_tr_per_s_sq))
  }};

  // Wrapping settings for sim
  if (config.closedLoop.positionWrappingEnabled.has_value()) {
    if (config.closedLoop.positionWrappingEnabled.value()) {
      _rioPidController.EnableContinuousInput(
          config.closedLoop.positionWrappingMinInput.value_or(0_tr).value(),
          config.closedLoop.positionWrappingMaxInput.value_or(0_tr).value());
    } else {
      _rioPidController.DisableContinuousInput();
    }
  }

  // Set the rio pid gains to match the spark config
  _rioPidController.SetPID(slot0.p.value_or(oldSlot0.p.value_or(0)),
                           slot0.i.value_or(oldSlot0.i.value_or(0)),
                           slot0.d.value_or(oldSlot0.d.value_or(0)));

  // Run the configuration and return any errors
  _configCache = config; // todo: overwrite individual settings, not whole lot.
  rev::spark::SparkBaseConfig revConfig;
  config.FillREVConfig(revConfig);
  return _spark->Configure(revConfig, resetMode, persistMode);
}

rev::REVLibError ICSpark::AdjustConfig(ICSparkConfig &config) {
  return Configure(config, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
                            rev::spark::SparkBase::PersistMode::kPersistParameters);
};

rev::REVLibError ICSpark::AdjustConfigNoPersist(ICSparkConfig &config) {
  return Configure(config, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
                            rev::spark::SparkBase::PersistMode::kNoPersistParameters);
};

rev::REVLibError ICSpark::OverwriteConfig(ICSparkConfig &config) {
  return Configure(config, rev::spark::SparkBase::ResetMode::kResetSafeParameters,
                            rev::spark::SparkBase::PersistMode::kPersistParameters);
};

void ICSpark::SetPosition(units::turn_t position) {
  _encoder.SetPosition(position.value());
}

void ICSpark::SetPositionTarget(units::turn_t target, units::volt_t arbFeedForward) {
  _positionTarget = target;
  _velocityTarget = units::turns_per_second_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  _latestModelFeedForward = CalculateFeedforward(target, 0_tps);
  _controlType = ControlType::kPosition;

  _sparkPidController.SetReference(target.value(), rev::spark::SparkLowLevel::ControlType::kPosition, rev::spark::ClosedLoopSlot::kSlot0,
                                   _arbFeedForward.value() + _latestModelFeedForward.value());
}

void ICSpark::SetMaxMotionTarget(units::turn_t target, units::volt_t arbFeedForward) {
  _positionTarget = target;
  _velocityTarget = units::turns_per_second_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  _latestMotionTarget = {GetPosition(), GetVelocity()};
  _controlType = ControlType::kMaxMotion;

  UpdateControls();
}

void ICSpark::SetMotionProfileTarget(units::turn_t target, units::volt_t arbFeedForward) {
  _positionTarget = target;
  _velocityTarget = units::turns_per_second_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  _latestMotionTarget = {GetPosition(), GetVelocity()};
  _controlType = ControlType::kMotionProfile;

  UpdateControls();
}

void ICSpark::SetVelocityTarget(units::turns_per_second_t target, units::volt_t arbFeedForward) {
  _velocityTarget = target;
  _positionTarget = units::turn_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = arbFeedForward;
  _latestModelFeedForward = CalculateFeedforward(0_tr, _velocityTarget);
  _controlType = ControlType::kVelocity;

  _sparkPidController.SetReference(target.value(), rev::spark::SparkLowLevel::ControlType::kVelocity, rev::spark::ClosedLoopSlot::kSlot0,
                                   _arbFeedForward.value() + _latestModelFeedForward.value());
}

void ICSpark::SetDutyCycle(double speed) {
  _velocityTarget = units::turns_per_second_t{0};
  _positionTarget = units::turn_t{0};
  _voltageTarget = 0_V;
  _arbFeedForward = 0_V;
  _latestModelFeedForward = 0_V;
  _controlType = ControlType::kDutyCycle;

  _simVoltage = std::clamp(speed, -1.0, 1.0) * _spark->GetBusVoltage() * 1_V;
  _sparkPidController.SetReference(speed, rev::spark::SparkLowLevel::ControlType::kDutyCycle);
}

void ICSpark::SetVoltage(units::volt_t output) {
  _velocityTarget = units::turns_per_second_t{0};
  _positionTarget = units::turn_t{0};
  _voltageTarget = output;
  _arbFeedForward = 0_V;
  _controlType = ControlType::kVoltage;

  _simVoltage = output;
  _sparkPidController.SetReference(output.value(), rev::spark::SparkLowLevel::ControlType::kVoltage);
}

void ICSpark::UpdateControls(units::second_t loopTime) {
  auto prevVelTarget = _latestMotionTarget.velocity;
  double sparkTarget = 0;

  switch (GetControlType()) {
    case ControlType::kMotionProfile:
      // In motion profile mode, we use the prev target state as the "current state"
      // and the sparkPIDController uses the next target state as its goal.
      _latestMotionTarget = CalcNextMotionTarget(_latestMotionTarget, _positionTarget, loopTime);
      sparkTarget = _latestMotionTarget.position.value();
      break;
    case ControlType::kMaxMotion: {
      // In Max Motion mode, we use the true, sensed current state state as the "current state"
      // and the sparkPIDController uses the overall target as its goal.
      MPState currentState = {GetPosition(), GetVelocity()};
      _latestMotionTarget = CalcNextMotionTarget(currentState, _positionTarget, loopTime);
      sparkTarget = _positionTarget.value();
      break;
    }
    case ControlType::kPosition:
      sparkTarget = _positionTarget.value();
      break;
    case ControlType::kVelocity:
      sparkTarget = _velocityTarget.value();
      break;
    default:
      return;
  }

  auto accelTarget = (_latestMotionTarget.velocity - prevVelTarget) / loopTime;
  _latestModelFeedForward =
      CalculateFeedforward(_latestMotionTarget.position, _latestMotionTarget.velocity, accelTarget);
  units::volt_t feedforward = _arbFeedForward + _latestModelFeedForward;
  _sparkPidController.SetReference(sparkTarget, GetREVControlType(), rev::spark::ClosedLoopSlot::kSlot0, feedforward.value());
}

units::volt_t ICSpark::CalculateFeedforward(units::turn_t pos, units::turns_per_second_t vel,
                                            units::turns_per_second_squared_t accel) {
  return _feedforwardStaticFriction * wpi::sgn(vel) + _feedforwardLinearGravity +
         _feedforwardRotationalGravity * units::math::cos(pos) + _feedforwardVelocity * vel +
         _feedforwardAcceleration * accel;
}

rev::spark::SparkLowLevel::ControlType ICSpark::GetREVControlType() {
  auto controlType = GetControlType();
  if (controlType == ControlType::kMotionProfile) {
    return rev::spark::SparkLowLevel::ControlType::kPosition;
  } else {
    return (rev::spark::SparkLowLevel::ControlType)controlType;
  }
}

void ICSpark::SetMotionMaxVel(units::turns_per_second_t maxVelocity) {
  ICSparkConfig config;
  config.closedLoop.slots[0].maxMotion.maxVelocity = maxVelocity;
  AdjustConfig(config);
}

void ICSpark::SetMotionMaxAccel(units::turns_per_second_squared_t maxAcceleration) {
  ICSparkConfig config;
  config.closedLoop.slots[0].maxMotion.maxAcceleration = maxAcceleration;
  AdjustConfig(config);
}

ICSparkConfig ICSpark::UseAbsoluteEncoder(units::turn_t zeroOffset) {
  _encoder.UseAbsolute(_spark->GetAbsoluteEncoder());

  // Create a config to help the user to setup an abolute encoder
  ICSparkConfig config;
  config.absoluteEncoder.averageDepth = 128;
  config.absoluteEncoder.zeroOffset = zeroOffset;
  config.signals.absoluteEncoderPositionAlwaysOn = true;
  config.signals.absoluteEncoderPositionPeriodMs = 10_ms;
  config.signals.absoluteEncoderVelocityAlwaysOn =true;
  config.signals.absoluteEncoderVelocityPeriodMs = 10_ms;
  config.closedLoop.feedbackSensor = rev::spark::ClosedLoopConfig::kAbsoluteEncoder;
  return config;
}

void ICSpark::SetFeedbackProportional(double P) {
  ICSparkConfig config;
  config.closedLoop.slots[0].p = P;
  AdjustConfig(config);
}

void ICSpark::SetFeedbackIntegral(double I) {
  ICSparkConfig config;
  config.closedLoop.slots[0].i = I;
  AdjustConfig(config);
}

void ICSpark::SetFeedbackDerivative(double D) {
  ICSparkConfig config;
  config.closedLoop.slots[0].d = D;
  AdjustConfig(config);
}

void ICSpark::SetFeedforwardGains(units::volt_t S, units::volt_t G, bool gravityIsRotational,
                                  VoltsPerTps V, VoltsPerTpsSq A, bool updateSparkNow) {
  SetFeedforwardStaticFriction(S, false);
  SetFeedforwardVelocity(V, false);
  SetFeedforwardAcceleration(A, false);
  if (gravityIsRotational) {
    SetFeedforwardRotationalGravity(G, false);
  } else {
    SetFeedforwardLinearGravity(G, false);
  }
  if (updateSparkNow) UpdateControls(0_s);
}

void ICSpark::SetFeedforwardStaticFriction(units::volt_t S, bool updateSparkNow) {
  _feedforwardStaticFriction = S;
  if (updateSparkNow) UpdateControls(0_s);
}

void ICSpark::SetFeedforwardLinearGravity(units::volt_t linearG, bool updateSparkNow) {
  _feedforwardLinearGravity = linearG;
  if (updateSparkNow) UpdateControls(0_s);
}

void ICSpark::SetFeedforwardRotationalGravity(units::volt_t rotationalG, bool updateSparkNow) {
  _feedforwardRotationalGravity = rotationalG;
  if (updateSparkNow) UpdateControls(0_s);
}

void ICSpark::SetFeedforwardVelocity(VoltsPerTps V, bool updateSparkNow) {
  _feedforwardVelocity = V;
  if (updateSparkNow) UpdateControls(0_s);
}

void ICSpark::SetFeedforwardAcceleration(VoltsPerTpsSq A, bool updateSparkNow) {
  _feedforwardAcceleration = A;
  if (updateSparkNow) UpdateControls(0_s);
}

units::turns_per_second_t ICSpark::GetVelocity() {
  return units::turns_per_second_t{_encoder.GetVelocity()};
}

double ICSpark::GetDutyCycle() const {
  if constexpr (frc::RobotBase::IsSimulation()) {
    return _simVoltage.value()/_spark->GetBusVoltage();
  } else {
    return _spark->GetAppliedOutput();
  }
}

units::volt_t ICSpark::GetMotorVoltage() {
  if constexpr (frc::RobotBase::IsSimulation()) {
    return _simVoltage;
  } else {
    return _spark->GetAppliedOutput() * _spark->GetBusVoltage()*1_V;
  }
}

units::volt_t ICSpark::CalcSimVoltage() {
  units::volt_t output = 0_V;
  // Allowing use of config accessor in the sim only functions
  // Don't use them in hardware code because they are blocking calls and wait on CAN bus responses.
  double posConversionFactor = _sparkConfigAccessor.encoder.GetPositionConversionFactor(); 
  double velConversionFactor = _sparkConfigAccessor.encoder.GetVelocityConversionFactor();

  switch (_controlType) {
    case ControlType::kDutyCycle:
      output = _spark->Get() * _spark->GetBusVoltage()*1_V;
      break;

    case ControlType::kVelocity:
      // Spark internal PID uses native units (motor shaft RPM)
      // so divide by conversion factor to use that
      output = units::volt_t{
          _rioPidController.Calculate(GetVelocity().value(), _velocityTarget.value()) /
          velConversionFactor};
      break;

    case ControlType::kPosition:
      // Spark internal PID uses native units (motor shaft rotations)
      // so divide by conversion factor to use that
      output = units::volt_t{
          _rioPidController.Calculate(GetPosition().value(), _positionTarget.value()) /
          posConversionFactor};
      break;

    case ControlType::kVoltage:
      output = _voltageTarget;
      break;

    case ControlType::kMaxMotion:
    case ControlType::kMotionProfile:
      output = units::volt_t{
          _rioPidController.Calculate(GetPosition().value(), _latestMotionTarget.position.value()) /
          posConversionFactor};
      break;

    case ControlType::kCurrent:
      break;
  }

  output += _arbFeedForward + _latestModelFeedForward;

  // Soft limits
  bool posLimitOn = _configCache.softLimit.forwardSoftLimitEnabled.value_or(false);
  bool negLimitOn = _configCache.softLimit.reverseSoftLimitEnabled.value_or(false);
  units::turn_t posLimit = _configCache.softLimit.forwardSoftLimit.value_or(0_tr);
  units::turn_t negLimit = _configCache.softLimit.reverseSoftLimit.value_or(0_tr);
  if (posLimitOn && GetPosition() >= posLimit && output > 0_V) {
    output = 0_V;
  }
  if (negLimitOn && GetPosition() <= negLimit && output < 0_V) {
    output = 0_V;
  }

  double minOutput = _configCache.closedLoop.slots[0].minOutput.value_or(-1);
  double maxOutput = _configCache.closedLoop.slots[0].maxOutput.value_or(1);
  output = std::clamp(output, minOutput * 12_V, maxOutput * 12_V);

  // store a latest copy because we can't call calculate() on the rio pid controller whenever we
  // want, it expects to be called at a specific frequency.
  _simVoltage = output;

  return output;
}

void ICSpark::IterateSim(units::turns_per_second_t velocity, units::volt_t batteryVoltage) {
  const units::second_t dt = 20_ms;
  _simSpark.iterate(velocity.value(), batteryVoltage.value(), dt.value());
}

bool ICSpark::InMotionMode() {
  return GetControlType() == ControlType::kMotionProfile ||
         GetControlType() == ControlType::kMaxMotion;
}

ICSpark::MPState ICSpark::CalcNextMotionTarget(MPState current, units::turn_t goalPosition,
                                               units::second_t lookahead) {
  units::turn_t error = units::math::abs(goalPosition - GetPosition());
  units::turn_t tolerance =
      _configCache.closedLoop.slots[0].maxMotion.allowedClosedLoopError.value_or(0_tr);
  if (error < tolerance) {
    return MPState{GetPosition(), 0_tps};
  }

  return _motionProfile.Calculate(
      lookahead, current,
      {goalPosition, units::turns_per_second_t{0}});
}
