#include "utilities/ICSpark.h"

#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <wpi/MathExtras.h>
#include <units/voltage.h>
#include <cstdlib>
#include <iostream>
#include <rev/ClosedLoopSlot.h>

ICSpark::ICSpark(rev::spark::SparkBase* spark, rev::spark::SparkRelativeEncoder& inbuiltEncoder,
                 rev::spark::SparkBaseConfigAccessor& configAccessor, units::ampere_t currentLimit)
    : _spark(spark),
      _sparkConfigAccessor(configAccessor),
      _encoder(inbuiltEncoder),
      _simSpark(spark, &_simMotor) {
  OverwriteConfig(_sparkConfig.SmartCurrentLimit(currentLimit.value()));
  SetConversionFactor(1);  // Makes the internal encoder use revs per sec not revs per min
}

void ICSpark::InitSendable(wpi::SendableBuilder& builder) {
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
  builder.AddDoubleProperty("Motion Config/Max vel",      [&] { return _motionConstraints.maxVelocity.value(); },    [&](double vel) { SetMotionMaxVel(vel*1_tps); });
  builder.AddDoubleProperty("Motion Config/Max accel",    [&] { return _motionConstraints.maxAcceleration.value(); },[&](double accel) { SetMotionMaxAccel(accel*1_tr_per_s_sq); });
  builder.AddDoubleProperty("OutputCurrent",              [&] { return GetMotorOutputCurrent().value(); },        nullptr);
  builder.AddDoubleProperty("Temperature",                [&] { return GetTemperature().value(); },               nullptr);
  // clang-format on
}

rev::REVLibError ICSpark::Configure(rev::spark::SparkBaseConfig& config,
                                    rev::spark::SparkBase::ResetMode resetMode,
                                    rev::spark::SparkBase::PersistMode persistMode) {
  // Run the configuration and save any errors
  auto err = _spark->Configure(config, resetMode, persistMode);

  // Cache the motion profile config for sim and feed forward model
  _motionProfileTolerance =
      _sparkConfigAccessor.closedLoop.maxMotion.GetAllowedClosedLoopError() * 1_tr;
  units::turns_per_second_t maxVelocity =
      _sparkConfigAccessor.closedLoop.maxMotion.GetMaxVelocity() * 1_tps;
  units::turns_per_second_squared_t maxAcceleration =
      _sparkConfigAccessor.closedLoop.maxMotion.GetMaxAcceleration() * 1_tr_per_s_sq;
  _motionConstraints = {maxVelocity, maxAcceleration};
  _motionProfile = frc::TrapezoidProfile<units::turns>{_motionConstraints};

  // Cache the wrapping settings for sim
  bool closedLoopEnabled = _sparkConfigAccessor.closedLoop.GetPositionWrappingEnabled();
  if (closedLoopEnabled) {
    _rioPidController.EnableContinuousInput(
        _sparkConfigAccessor.closedLoop.GetPositionWrappingMinInput(),
        _sparkConfigAccessor.closedLoop.GetPositionWrappingMaxInput());
  } else {
    _rioPidController.DisableContinuousInput();
  }

  // Cache the min and max closed loop outputs for sim
  _minClosedLoopOutputCache = _sparkConfigAccessor.closedLoop.GetMinOutput();
  _maxClosedLoopOutputCache = _sparkConfigAccessor.closedLoop.GetMaxOutput();

  // Set the rio pid gains to match the spark config
  _rioPidController.SetPID(_sparkConfigAccessor.closedLoop.GetP(),
                           _sparkConfigAccessor.closedLoop.GetI(),
                           _sparkConfigAccessor.closedLoop.GetD());

  return err;
}

rev::REVLibError ICSpark::AdjustConfig(rev::spark::SparkBaseConfig &config) {
  return Configure(config, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
                            rev::spark::SparkBase::PersistMode::kPersistParameters);
};

rev::REVLibError ICSpark::AdjustConfigNoPersist(rev::spark::SparkBaseConfig &config) {
  return Configure(config, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
                            rev::spark::SparkBase::PersistMode::kNoPersistParameters);
};

rev::REVLibError ICSpark::OverwriteConfig(rev::spark::SparkBaseConfig &config) {
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

void ICSpark::SetMotionConstraints(units::turns_per_second_t maxVelocity,
                                   units::turns_per_second_squared_t maxAcceleration,
                                   units::turn_t tolerance) {
  _sparkConfig.closedLoop.maxMotion.MaxVelocity(maxVelocity.value())
      .MaxAcceleration(maxAcceleration.value());
  AdjustConfigWithoutCache(_sparkConfig);
  _motionConstraints = {maxVelocity, maxAcceleration};
  _motionProfileTolerance = tolerance;
  _motionProfile = frc::TrapezoidProfile<units::turns>{_motionConstraints};
}

void ICSpark::SetMotionMaxVel(units::turns_per_second_t maxVelocity) {
  auto accel = _motionConstraints.maxAcceleration;
  SetMotionConstraints(maxVelocity, accel, _motionProfileTolerance);
}

void ICSpark::SetMotionMaxAccel(units::turns_per_second_squared_t maxAcceleration) {
  auto vel = _motionConstraints.maxVelocity;
  SetMotionConstraints(vel, maxAcceleration, _motionProfileTolerance);
}

void ICSpark::SetConversionFactor(double rotationsToDesired) {
  _sparkConfig.encoder
    .PositionConversionFactor(1.0 / rotationsToDesired)
    .VelocityConversionFactor(rotationsToDesired / 60);
  _sparkConfig.absoluteEncoder
    .PositionConversionFactor(1.0 / rotationsToDesired)
    .VelocityConversionFactor(rotationsToDesired / 60);
  _sparkConfig.analogSensor
    .PositionConversionFactor(1.0 / rotationsToDesired)
    .VelocityConversionFactor(rotationsToDesired / 60);
  AdjustConfig(_sparkConfig);
}

void ICSpark::UseAbsoluteEncoder(units::turn_t zeroOffset) {
  _encoder.UseAbsolute(_spark->GetAbsoluteEncoder());

  // Config the spark's abolute encoder
  _sparkConfig.absoluteEncoder.AverageDepth(128).ZeroOffset(zeroOffset.value());
  _sparkConfig.signals.AbsoluteEncoderPositionAlwaysOn(true)
      .AbsoluteEncoderPositionPeriodMs(10)
      .AbsoluteEncoderVelocityAlwaysOn(true)
      .AbsoluteEncoderVelocityPeriodMs(10);
  _sparkConfig.closedLoop.SetFeedbackSensor(
      rev::spark::ClosedLoopConfig::FeedbackSensor::kAbsoluteEncoder);

  AdjustConfig(_sparkConfig);
}

void ICSpark::EnableClosedLoopWrapping(units::turn_t min, units::turn_t max) {
  _sparkConfig.closedLoop.PositionWrappingMinInput(min.value())
      .PositionWrappingMaxInput(max.value())
      .PositionWrappingEnabled(true);
  _spark->Configure(_sparkConfig, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
                    rev::spark::SparkBase::PersistMode::kPersistParameters);

  _rioPidController.EnableContinuousInput(min.value(), max.value());
}

void ICSpark::SetFeedbackGains(double P, double I, double D) {
  SetFeedbackProportional(P);
  SetFeedbackIntegral(I);
  SetFeedbackDerivative(D);
}

void ICSpark::SetFeedbackProportional(double P) {
  _sparkConfig.closedLoop.P(P);
  AdjustConfigWithoutCache(_sparkConfig);
  _rioPidController.SetP(P);
}

void ICSpark::SetFeedbackIntegral(double I) {
  _sparkConfig.closedLoop.I(I);
  AdjustConfigWithoutCache(_sparkConfig);
  _rioPidController.SetI(I);
}

void ICSpark::SetFeedbackDerivative(double D) {
  _sparkConfig.closedLoop.D(D);
  AdjustConfigWithoutCache(_sparkConfig);
  _rioPidController.SetD(D);
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

units::ampere_t ICSpark::GetMotorOutputCurrent() {
  return _spark->GetOutputCurrent()*1_A;
}

units::celsius_t ICSpark::GetTemperature() {
  return _spark->GetMotorTemperature() * 1_degC;
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
  bool posLimitOn = _sparkConfigAccessor.softLimit.GetForwardSoftLimitEnabled();
  bool negLimitOn = _sparkConfigAccessor.softLimit.GetReverseSoftLimitEnabled();
  double posLimit = _sparkConfigAccessor.softLimit.GetForwardSoftLimit();
  double negLimit = _sparkConfigAccessor.softLimit.GetReverseSoftLimit();
  if (posLimitOn && GetPosition().value() >= posLimit && output > 0_V) {
    output = 0_V;
  }
  if (negLimitOn && GetPosition().value() <= negLimit && output < 0_V) {
    output = 0_V;
  }

  output = std::clamp(output, _minClosedLoopOutputCache * 12_V, _maxClosedLoopOutputCache * 12_V);

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
  if (error < _motionProfileTolerance) {
    return MPState{GetPosition(), 0_tps};
  }

  return _motionProfile.Calculate(
      lookahead, current,
      {goalPosition, units::turns_per_second_t{0}});
}
