#pragma once

#include <rev/SparkBase.h>
#include <rev/SparkSim.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/simulation/SimDeviceSim.h>
#include <hal/SimDevice.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/time.h>
#include <units/current.h>
#include <units/velocity.h>
#include <units/temperature.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include "utilities/ICSparkEncoder.h"


/**
 * Wrapper around the Rev CANSparkBase class with some convenience features.
 * - Required Current Limit Setting
 * - Better simulation support (see CalcSimVoltage() and IterateSim())
 * - Uses C++ units
 * - Encoder and pid functions are built into this class
 */
class ICSpark : public wpi::Sendable {
 public:
  enum class ControlType {
    kDutyCycle = (int)rev::spark::SparkLowLevel::ControlType::kDutyCycle,
    kVelocity = (int)rev::spark::SparkLowLevel::ControlType::kVelocity,
    kVoltage = (int)rev::spark::SparkLowLevel::ControlType::kVoltage,
    kPosition = (int)rev::spark::SparkLowLevel::ControlType::kPosition,
    kMaxMotion = (int)rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl,
    kCurrent = (int)rev::spark::SparkLowLevel::ControlType::kCurrent,
    kMotionProfile = 10
  };

  using VoltsPerTps = units::unit_t<units::compound_unit<
      units::volts, units::inverse<units::turns_per_second>>>;
  using VoltsPerTpsSq = units::unit_t<units::compound_unit<
      units::volts, units::inverse<units::turns_per_second_squared>>>;

  /**
   * Create a new object to control a SPARK motor controller.
   *
   * @param currentLimit Value used for spark smart current limiting
   * @param inbultEncoder rvalue reference to the encoder built into the NEO
   * @param spark Reference to the spark to control
   */
  ICSpark(rev::spark::SparkBase* spark, rev::spark::SparkRelativeEncoder& inbuiltEncoder,
          rev::spark::SparkBaseConfigAccessor& configAccessor, units::ampere_t currentLimit);

  /**
   * Sets position of motor
   *
   * @param position What to set the position to
   */
  void SetPosition(units::turn_t position);

  /**
   * Sets a closed loop position target (aka reference or goal) for the motor to drive to.
   *
   * @param target The target position drive to.
   *
   * @param arbFeedforward A voltage from -32.0V to 32.0V which is applied to the motor after the
   * result of the specified control mode. This value is added after the control mode, but before
   * any current limits or ramp rates
   */
  void SetPositionTarget(units::turn_t target, units::volt_t arbFeedForward = 0.0_V);

  /**
   * Sets a closed loop position target (aka reference or goal) for the motor to
   * drive to using the Spark Max's Smart Motion control mode. This generates a
   * profiled movement that accelerates and decelerates in a controlled way.
   * This can reduce ware on components, limit current draw and is can be easier
   * to tune. In this mode, you are actually controlling the velocity of the
   * motor to follow a trapezoid (speeding up, staying constant, then slowing
   * down) and as such the PID values should be tuned to follow a velocity
   * target. Also consider using SetMotionProfileTarget() to compute a profile
   * on the RoboRio and perform feedback control based on position.
   *
   * @param target The target position drive to.
   *
   * @param arbFeedforward A voltage from -32.0V to 32.0V which is applied to
   * the motor after the result of the specified control mode. This value is
   * added after the control mode, but before any current limits or ramp rates
   */
  void SetMaxMotionTarget(units::turn_t target, units::volt_t arbFeedForward = 0.0_V);

  /**
   * Sets a closed loop position target (aka reference or goal) for the motor to
   * drive to using a motion profile computed onboard the RoboRio and followed
   * using the Spark Max's onboard PID position control.
   * This generates a profiled movement that accelerates and decelerates in a controlled way. This
   * can reduce ware on components, limit current draw and can be easier to tune.
   * 
   */
  void SetMotionProfileTarget(units::turn_t target, units::volt_t arbFeedForward = 0.0_V);

  /**
   * Sets the closed loop target (aka reference or goal) for the motor to drive to.
   *
   * @param target The target position drive to.
   *
   * @param arbFeedforward A voltage from -32.0V to 32.0V which is applied to the motor after the
   * result of the specified control mode. This value is added after the control mode, but before
   * any current limits or ramp rates
   */
  void SetVelocityTarget(units::turns_per_second_t target, units::volt_t arbFeedForward = 0.0_V);

  /**
   * Update motion profile targets and feedforward calculations. This is required to be called
   * periodically when you use the Motion Profile control type or any feedforward gains.
   *
   * @param loopTime The frequency at which this is being called. 20ms is the default loop time for
   * WPILib periodic functions.
   */
  void UpdateControls(units::second_t loopTime = 20_ms);

  /**
   * Calculate how many volts to send to the motor from the feedforward model configured with
   * SetFeedforwardGains().
   *
   * @param pos The position target
   * @param vel The velocity target
   * @param accel The acceleration target
   */
  units::volt_t CalculateFeedforward(units::turn_t pos, units::turns_per_second_t vel,
                                     units::turns_per_second_squared_t accel = 0_tr_per_s_sq);

  /**
   * Gets the current closed loop position target if there is one. Zero otherwise.
   */
  units::turn_t GetPositionTarget() { return _positionTarget; };

  /**
   * Gets the current closed loop velocity target if there is one. Zero otherwise
   */
  units::turns_per_second_t GetVelocityTarget() { return _velocityTarget; };

  /**
   * Get the closed loop position error (current position - target position) if there is one.
   * Zero otherwise.
   */
  units::turn_t GetPosError() { return GetPosition() - _positionTarget; }

  /**
   * Get the closed loop velocity error (current velocity - target velocity) if there is one.
   * Zero otherwise.
   */
  units::turns_per_second_t GetVelError() { return GetVelocity() - _velocityTarget; }

  /**
   * Calculates how much voltage the spark max would be giving to the attached motor given its
   * current control type and PID configuration. Use this in conjunction with one of WPILib's
   * physics simulation classes.
   * (https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html)
   */
  units::volt_t CalcSimVoltage();

  /**
   * Run internal calculations and set internal state.
   * This method belongs in Simulation Periodic. Use a WPILib physics simulation class or equivalent
   * to calculate the velocity from the applied output and pass it in to this method, which will
   * update the simulated state of the motor.
   *
   * Simulating a Spark this way will use the configurations and controls of the original
   * CANSparkMax or CANSparkFlex device to simulate velocity noise, all supported control modes
   * (including MAXMotion), arb feedforward input, voltage compensation, limit switches, soft
   * limits, and current limiting, with algorithms translated directly from the Spark firmware.
   *
   * This method will update the CANSparkSim's position and velocity, accessible with getPosition()
   * and getVelocity(). These values are automatically used as the selected feedback sensor for
   * calculations like closed-loop control and soft limits, and are reflected in the selected
   * sensor's value. Other sensors each have their own Sim class, which can be used to inject their
   * positions based on these calculations, to match how they are configured physically. For
   * example, to represent an Absolute Encoder on a 1:5 ratio from the mechanism,
   * SparkAbsoluteEncoderSim.iterate(double, double) is called each simulationPeriodic loop with a
   * velocity divided by 5. The selected sensor's position and velocity will automatically be
   * updated to match the CANSparkSim's when this method is called.
   *
   * Parameters:
   * @param velocity - The externally calculated velocity in units after conversion. The internal
   * simulation state will 'lag' slightly behind this input due to the SPARK Device internal
   * filtering. Noise will also be added.
   * @param vbus - Bus voltage in volts (See WPILib's BatterySim class to simulate this, or use 12V)
   * @param dt - Simulation time step in seconds
   */
  void IterateSim(units::turns_per_second_t velocity, units::volt_t batteryVoltage = 12_V);

  /**
   * Gets the current closed loop control type.
   */
  ControlType GetControlType() { return _controlType; };

  /**
   * Gets the output current.
   */
  units::ampere_t GetMotorOutputCurrent();

  /**
   * Gets motors temperature.
   */
  units::celsius_t GetTemperature();
  /**
   * Get the velocity of the motor.
   */
  units::turns_per_second_t GetVelocity();

  /**
   * Get the position of the motor.
   */
  units::turn_t GetPosition() { return units::turn_t{_encoder.GetPosition()}; };

  /**
   * Get the duty cycle (-1 to 1) of the motor.
  */
  double GetDutyCycle() const;

  /**
   * Get the voltage applied to the motor.
  */
  units::volt_t GetMotorVoltage();
  
  /**
   * Common interface to stop the motor until Set is called again or closed loop control is started.
   */
  void StopMotor() { SetDutyCycle(0); };

  /**
   * Sets the duty cycle of a speed controller.
   *
   * @param speed The duty cycle to set. Value should be between -1.0 and 1.0.
   */
  void SetDutyCycle(double speed);

  /**
   * Sets the voltage of a speed controller.
   *
   * @param output The voltage to set.
   */
  void SetVoltage(units::volt_t output);

  /**
   * Configure the constrains for the SparkMax's Smart Motion control mode. Maximum velocity,
   * maximum acceleration, and position tolerance must be set.
   *
   * @param maxVelocity The maxmimum velocity for the motion profile.
   * @param maxAcceleration The maxmimum acceleration for the motion profile.
   * @param tolerance When the position of the motor is within tolerance, the control mode will stop
   * applying power (arbitary feedforward can still apply power).
   */
  void SetMotionConstraints(units::turns_per_second_t maxVelocity,
                            units::turns_per_second_squared_t maxAcceleration,
                            units::turn_t tolerance);
  void SetMotionMaxVel(units::turns_per_second_t maxVelocity);
  void SetMotionMaxAccel(units::turns_per_second_squared_t maxAcceleration);

  /**
   * Set the conversion factor for position, velocity and acceleration of the encoder. The native
   * position units of rotations will be multipled by this number before being used or returned.
   * Velocity and acceleration conversion factors will be derived from this (as position units per
   * second for velocity and velocity units per second for acceleration)
   */
  void SetConversionFactor(double rotationsToDesired);

  /**
   * Set the Proportional, Integral, Derivative and static FeedForward gain constants of the PID
   * controller on the SPARK. This uses the Set Parameter API and should be used infrequently.
   * The parameters do not presist unless burnFlash() is called.
   *
   * @param P The proportional gain value, must be positive
   * @param I The Integral gain value, must be positive
   * @param D The Derivative gain value, must be positive
   */
  void SetFeedbackGains(double P, double I, double D);
  void SetFeedbackProportional(double P);
  void SetFeedbackIntegral(double I);
  void SetFeedbackDerivative(double D);

  /**
   * Set the Static Friction, Gravity, Velocity and Acceleration gain constants of the feed forward
   * model. This uses the Set Parameter API and should be used infrequently. The parameters do not
   * presist unless burnFlash() is called.
   *
   * @param P The proportional gain value, must be positive
   * @param I The Integral gain value, must be positive
   * @param D The Derivative gain value, must be positive
   * @param updateSparkNow Whether to calculate and send the new FF voltage to the spark over CAN as
   * part of this call. This will use the (blocking) Set Parameter API of the Spark and should be
   * used infrequently. The parameters do not presist unless burnFlash() is called. 
   */
  void SetFeedforwardGains(units::volt_t S = 0_V, units::volt_t G = 0_V,
                           bool gravityIsRotational = false, VoltsPerTps V = 0_V / 1_tps,
                           VoltsPerTpsSq A = 0_V / 1_tr_per_s_sq, bool updateSparkNow = true);
  void SetFeedforwardStaticFriction(units::volt_t S, bool updateSparkNow = true);
  void SetFeedforwardLinearGravity(units::volt_t linearG, bool updateSparkNow = true);
  void SetFeedforwardRotationalGravity(units::volt_t rotationalG, bool updateSparkNow = true);
  void SetFeedforwardVelocity(VoltsPerTps V, bool updateSparkNow = true);
  void SetFeedforwardAcceleration(VoltsPerTpsSq A, bool updateSparkNow = true);

  /**
   * Switch to using an external absolute encoder connected to the data port on
   * the SPARK. To use a relative encoder, check the Spark Max and Spark Flex
   * specific implimentation. The Max uses "Alternate encoders" while the Flex
   * uses "External encoders"
   *
   * @param zeroOffset the position that is reported as zero. It is influenced
   * by the absolute encoder's position conversion factor, and whether it is
   * inverted. So set those parameters before calling this.
   */
  void UseAbsoluteEncoder(units::turn_t zeroOffset = 0_tr);

  /**
   * Set the minimum and maximum input value for PID Wrapping with position closed loop
   * control.
   *
   * @param max The maximum input value
   * @param min The minimum input value
   */
  void EnableClosedLoopWrapping(units::turn_t min, units::turn_t max);

  /**
   * Check whether the motor is on its position target, within a given tolerance.
   *
   * @param tolerance The tolerance to be considered on target
   */
  bool OnPosTarget(units::turn_t tolerance) { return units::math::abs(GetPosError()) < tolerance; }

  /**
   * Check whether the motor is on its velocity target, within a given tolerance.
   *
   * @param tolerance The tolerance to be considered on target
   */
  bool OnVelTarget(units::turns_per_second_t tolerance) {
    return units::math::abs(GetVelError()) < tolerance;
  }

  /**
   * Set the configuration for the SPARK.
   * When configuring the conversion factors, the ICSpark assumes you are converting position into
   * rotations and velocity into turns per second. Make sure to check whether your velocity
   * coversion factor needs to be divided by 60 to transform RPM into tps!
   *
   * If @c resetMode is ResetMode::kResetSafeParameters, this method will reset safe writable
   * parameters to their default values before setting the given configuration. The following
   * parameters will not be reset by this action: CAN ID, Motor Type, Idle Mode, PWM Input Deadband,
   * and Duty Cycle Offset.
   *
   * If @c persistMode is PersistMode::kPersistParameters, this method will save all parameters
   * to the SPARK's non-volatile memory after setting the given configuration. This will allow
   * parameters to persist across power cycles.
   *
   * @param config The desired SPARK configuration
   * @param resetMode Whether to reset safe parameters before setting the configuration
   * @param persistMode Whether to persist the parameters after setting the configuration
   * @return REVLibError::kOk if successful
   */
  rev::REVLibError Configure(rev::spark::SparkBaseConfig& config,
                             rev::spark::SparkBase::ResetMode resetMode,
                             rev::spark::SparkBase::PersistMode persistMode);

  /**  
   * Convenience method for calling 
   * Configure(config, ResetMode::kNoResetSafeParameters, PersistMode::kPersistParameters)
   */
  rev::REVLibError AdjustConfig(rev::spark::SparkBaseConfig &config);

  /**  
   * Convenience method for calling 
   * Configure(config, ResetMode::kNoResetSafeParameters, PersistMode::kNoPersistParameters)
   */
  rev::REVLibError AdjustConfigNoPersist(rev::spark::SparkBaseConfig &config);

  /**  
   * Convenience method for calling 
   * Configure(config, ResetMode::kResetSafeParameters, PersistMode::kPersistParameters)
   */
  rev::REVLibError OverwriteConfig(rev::spark::SparkBaseConfig &config);


  // Sendable setup, called automatically when this is passed into smartDashbaord::PutData()
  void InitSendable(wpi::SendableBuilder& builder) override;
  
 protected:
  // Use a relative (alternarte for Max, external for Flex) encoder as the feedback device.
  template <std::derived_from<rev::RelativeEncoder> RelEncoder>
  void UseRelativeEncoder(RelEncoder& encoder, int countsPerRev) {
    _encoder.UseRelative(encoder);
    _sparkConfig.closedLoop.SetFeedbackSensor(
        rev::spark::ClosedLoopConfig::FeedbackSensor::kAlternateOrExternalEncoder);
    AdjustConfig(_sparkConfig);
  }

 private:
  rev::spark::SparkBase* _spark;
  rev::spark::SparkBaseConfigAccessor _sparkConfigAccessor;
  rev::spark::SparkBaseConfig _sparkConfig;
  rev::REVLibError AdjustConfigWithoutCache(rev::spark::SparkBaseConfig& config) {
    return _spark->Configure(config, rev::spark::SparkBase::ResetMode::kNoResetSafeParameters,
                             rev::spark::SparkBase::PersistMode::kPersistParameters);
  }

  // Feedback control objects
  rev::spark::SparkClosedLoopController _sparkPidController{_spark->GetClosedLoopController()};
  frc::PIDController _rioPidController{0, 0, 0};
  ICSparkEncoder _encoder;

  // Feedforward gains
  units::volt_t _feedforwardStaticFriction = 0_V;
  units::volt_t _feedforwardLinearGravity = 0_V;
  units::volt_t _feedforwardRotationalGravity = 0_V;
  VoltsPerTps _feedforwardVelocity = 0_V / 1_tps;
  VoltsPerTpsSq _feedforwardAcceleration = 0_V / 1_tr_per_s_sq;
  units::volt_t _arbFeedForward = 0.0_V;
  units::volt_t _latestModelFeedForward = 0.0_V;

  // Control References (Targets)
  using MPState = frc::TrapezoidProfile<units::turns>::State;
  units::turn_t _positionTarget{0};
  units::turns_per_second_t _velocityTarget{0};
  units::volt_t _voltageTarget{0};
  frc::TrapezoidProfile<units::angle::turns>::Constraints _motionConstraints{
      units::turns_per_second_t{0},
      units::turns_per_second_squared_t{0}
  }; // constraints updated by SetMotionConstraints() or Configure()
  frc::TrapezoidProfile<units::turns> _motionProfile{_motionConstraints};
  MPState CalcNextMotionTarget(MPState current, units::turn_t goalPosition,
                               units::second_t lookahead = 20_ms);
  MPState _latestMotionTarget;

  // Control Type management
  ControlType _controlType = ControlType::kDutyCycle;
  rev::spark::SparkLowLevel::ControlType GetREVControlType();
  bool InMotionMode();

  // Store a cache of some of the config values since requesting them causes slow, blocking CAN
  // calls.
  double _minClosedLoopOutputCache = -1;
  double _maxClosedLoopOutputCache = 1;
  units::turn_t _motionProfileTolerance = 0_tr;

  // Simulation objects
  frc::DCMotor _simMotor = frc::DCMotor::NeoVortex();
  rev::spark::SparkSim _simSpark;
  // store a latest copy of sim voltage because we can't call calculate() on the pid controller
  // whenever we want, it expects to be called at a specific frequency.
  units::volt_t _simVoltage = 0_V;  
};
