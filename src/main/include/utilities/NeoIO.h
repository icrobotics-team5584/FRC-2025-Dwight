#include "IOSwerve.h"
#include "utilities/ICSparkMax.h"
#include "Constants.h"
#include <ctre/phoenix6/CANcoder.hpp>

class NeoIO : public SwerveIO{
  public:
    NeoIO(int turnCanID, int driveCanID, int encoderCanID, units::turn_t cancoderMagOffset);
    void ConfigTurnMotor() override;
    void SetDesiredAngle(units::degree_t angle) override;
    void SetAngle(units::turn_t angle) override;
    void SendSensorsToDash() override;
    void SetDesiredVelocity(units::meters_per_second_t velocity, units::newton_t forceFF) override;
    void DriveStraightVolts(units::volt_t volts) override;
    void StopMotors() override;
    void UpdateSim(units::second_t deltaTime) override;
    void SetNeutralMode(bool brakeModeToggle) override;
    void ConfigDriveMotor() override;
    frc::SwerveModulePosition GetPosition() override;
    frc::Rotation2d GetAngle() override;
    units::meters_per_second_t GetSpeed() override;
    units::volt_t GetDriveVoltage() override;
    frc::SwerveModuleState GetState() override;
    units::radian_t GetDrivenRotations() override;

    const double TURNING_GEAR_RATIO = 150.0 / 7.0;
    const double DRIVE_GEAR_RATIO = 6.75; // L2 - Fast kit
    const units::meter_t WHEEL_RADIUS = 0.0481098886_m;
    const units::meter_t WHEEL_CIRCUMFERENCE = 2 * std::numbers::pi * WHEEL_RADIUS;

    const double TURN_P = 20.0;
    const double TURN_I = 0.0;
    const double TURN_D = 0;
    const double DRIVE_P = 0.02;
    const double DRIVE_I = 0.0;
    const double DRIVE_D = 0.0;
    const double DRIVE_FF = 0.12; // CHANGE AND UPDATE FOR NEW ICSPARKMAX
  private:

    ICSparkMax _canTurnMotor;
    ICSparkMax _canDriveMotor;
    ctre::phoenix6::hardware::CANcoder _canEncoder;

};
