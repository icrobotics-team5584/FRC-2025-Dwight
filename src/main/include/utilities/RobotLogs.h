#include <ctre/phoenix6/TalonFX.hpp>
#include <frc/DriverStation.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/voltage.h>
#include <units/current.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/time.h>
#include <units/torque.h>
#include <units/mass.h>
#include <units/temperature.h>
#include <array>

#include <frc2/command/CommandScheduler.h>

namespace Logger {
void LogFalcon(std::string name, ctre::phoenix6::hardware::TalonFX& talonFX);

void Log(std::string_view keyName, wpi::Sendable* data);
void Log(std::string_view keyName, double value);
void Log(std::string_view keyName, int value);
void Log(std::string_view keyName, ctre::phoenix6::StatusSignal<double>& signal);
void Log(std::string_view keyName, bool value);
void Log(std::string_view keyName, std::string_view value);
void Log(std::string_view keyName, frc::DriverStation::Alliance value);
void Log(std::string_view keyName, wpi::array<frc::SwerveModuleState, 4> states);
void Log(std::string keyName, units::turn_t value);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::turn_t>& signal);
void Log(std::string keyName, units::degree_t value);
void Log(std::string keyName, units::turns_per_second_t value);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::turns_per_second_t>& signal);
void Log(std::string keyName, units::turns_per_second_squared_t value);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::turns_per_second_squared_t>& signal);
void Log(std::string keyName, units::meter_t value);
void Log(std::string keyName, units::meters_per_second_t value);
void Log(std::string keyName, units::meters_per_second_squared_t value);
void Log(std::string keyName, units::volt_t value);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::volt_t>& signal);
void Log(std::string keyName, units::ampere_t value);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::ampere_t>& signal);
void Log(std::string keyName, units::second_t value);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::second_t>& signal);
void Log(std::string keyName, units::kilogram_t value);
void Log(std::string keyName, units::celsius_t value);
void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::celsius_t>& signal);
void Log(std::string keyName, frc::Rotation2d value);

double Tune(std::string keyName, double defaultValue);
bool Tune(std::string keyName, bool defaultValue);
std::string Tune(std::string keyName, std::string defaultValue);
units::turn_t Tune(std::string keyName, units::turn_t defaultValue);
units::degree_t Tune(std::string keyName, units::degree_t defaultValue);
units::turns_per_second_t Tune(std::string keyName, units::turns_per_second_t defaultValue);
units::turns_per_second_squared_t Tune(std::string keyName, units::turns_per_second_squared_t defaultValue);
units::meter_t Tune(std::string keyName, units::meter_t defaultValue);
units::meters_per_second_t Tune(std::string keyName, units::meters_per_second_t defaultValue);
units::meters_per_second_squared_t Tune(std::string keyName, units::meters_per_second_squared_t defaultValue);
units::volt_t Tune(std::string keyName, units::volt_t defaultValue);
units::ampere_t Tune(std::string keyName, units::ampere_t defaultValue);
units::second_t Tune(std::string keyName, units::second_t defaultValue);
units::kilogram_t Tune(std::string keyName, units::kilogram_t defaultValue);
units::celsius_t Tune(std::string keyName, units::celsius_t defaultValue);
frc::Rotation2d Tune(std::string keyName, frc::Rotation2d defaultValue);

};  // namespace Logger