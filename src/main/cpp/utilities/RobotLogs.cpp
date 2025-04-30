#include "utilities/RobotLogs.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/StructArrayTopic.h>
#include <frc/kinematics/struct/SwerveModuleStateStruct.h>
#include <map>

namespace Logger {
std::map<std::string, nt::StructArrayPublisher<frc::SwerveModuleState>> swerveModuleStatePublishers;

void LogFalcon(std::string name, ctre::phoenix6::hardware::TalonFX& talonFX) {
  Log(name + "/Voltage", talonFX.GetMotorVoltage());
  Log(name + "/StatorCurrent", talonFX.GetStatorCurrent());
  Log(name + "/SupplyCurrent", talonFX.GetSupplyCurrent());
  Log(name + "/Position", talonFX.GetPosition());
  Log(name + "/Temperature", talonFX.GetDeviceTemp());
  Log(name + "/TrueVelocity", talonFX.GetVelocity());
  Log(name + "/Target", talonFX.GetClosedLoopReference());
}

void Log(std::string_view keyName, wpi::Sendable* data) {
  frc::SmartDashboard::PutData(keyName, data);
}

void Log(std::string_view keyName, double value) {
  frc::SmartDashboard::PutNumber(keyName, value);
}

void Log(std::string_view keyName, int value) {
  frc::SmartDashboard::PutNumber(keyName, value);
}

void Log(std::string_view keyName, ctre::phoenix6::StatusSignal<double>& signal) {
  Log(keyName, signal.GetValue());
}

void Log(std::string_view keyName, bool value) {
  frc::SmartDashboard::PutBoolean(keyName, value);
}

void Log(std::string_view keyName, std::string_view value) {
  frc::SmartDashboard::PutString(keyName, value);
}

void Log(std::string_view keyName, frc::DriverStation::Alliance value) {
  Log(keyName, value == frc::DriverStation::Alliance::kRed ? "Red" : "Blue");
}

void Log(std::string_view keyName, wpi::array<frc::SwerveModuleState, 4> value) {
  std::string fullKeyName = "SmartDashboard/" + std::string(keyName);
  auto [it, inserted] = swerveModuleStatePublishers.try_emplace(
      std::string(fullKeyName), nt::NetworkTableInstance::GetDefault()
                                    .GetStructArrayTopic<frc::SwerveModuleState>(fullKeyName)
                                    .Publish());
  it->second.Set(value);
}

void Log(std::string keyName, units::turn_t value) {
  Log(keyName + " (tr)", value.value());
}

void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::turn_t>& signal) {
  Log(keyName, signal.GetValue());
}

void Log(std::string keyName, units::degree_t value) {
  Log(keyName + " (deg)", value.value());
}

void Log(std::string keyName, units::turns_per_second_t value) {
  Log(keyName + " (tps)", value.value());
}

void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::turns_per_second_t>& signal) {
  Log(keyName, signal.GetValue());
}

void Log(std::string keyName, units::turns_per_second_squared_t value) {
  Log(keyName + " (tps_sq)", value.value());
}

void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::turns_per_second_squared_t>& signal) {
  Log(keyName, signal.GetValue());
}

void Log(std::string keyName, units::meter_t value) {
  Log(keyName + " (m)", value.value());
}

void Log(std::string keyName, units::meters_per_second_t value) {
  Log(keyName + " (mps)", value.value());
}

void Log(std::string keyName, units::meters_per_second_squared_t value) {
  Log(keyName + " (mps_sq)", value.value());
}

void Log(std::string keyName, units::volt_t value) {
  Log(keyName + " (V)", value.value());
}

void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::volt_t>& signal) {
  Log(keyName, signal.GetValue());
}

void Log(std::string keyName, units::ampere_t value) {
  Log(keyName + " (A)", value.value());
}

void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::ampere_t>& signal) {
  Log(keyName, signal.GetValue());
}

void Log(std::string keyName, units::second_t value) {
  Log(keyName + " (s)", value.value());
}

void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::second_t>& signal) {
  Log(keyName, signal.GetValue());
}

void Log(std::string keyName, units::kilogram_t value) {
  Log(keyName + " (kg)", value.value());
}

void Log(std::string keyName, units::celsius_t value) {
  Log(keyName + " (C)", value.value());
}

void Log(std::string keyName, ctre::phoenix6::StatusSignal<units::celsius_t>& signal) {
  Log(keyName, signal.GetValue());
}

void Log(std::string keyName, frc::Rotation2d value) {
  Log(keyName, value.Degrees());
}


double Tune(std::string keyName, double defaultValue) {
  if (frc::SmartDashboard::ContainsKey(keyName)) {
    return frc::SmartDashboard::GetNumber(keyName, defaultValue);
  } else {
    frc::SmartDashboard::PutNumber(keyName, defaultValue);
    return defaultValue;
  }
}

bool Tune(std::string keyName, bool defaultValue) {
  if (frc::SmartDashboard::ContainsKey(keyName)) {
    return frc::SmartDashboard::GetBoolean(keyName, defaultValue);
  } else {
    frc::SmartDashboard::PutBoolean(keyName, defaultValue);
    return defaultValue;
  }
}

std::string Tune(std::string keyName, std::string defaultValue) {
  if (frc::SmartDashboard::ContainsKey(keyName)) {
    return frc::SmartDashboard::GetString(keyName, defaultValue);
  } else {
    frc::SmartDashboard::PutString(keyName, defaultValue);
    return defaultValue;
  }
}

units::turn_t Tune(std::string keyName, units::turn_t defaultValue) {
  return units::turn_t(Tune(keyName + " (deg)", defaultValue.value()));
}

units::degree_t Tune(std::string keyName, units::degree_t defaultValue) {
  return units::degree_t(Tune(keyName + " (deg)", defaultValue.value()));
}

units::turns_per_second_t Tune(std::string keyName, units::turns_per_second_t defaultValue) {
  return units::turns_per_second_t(Tune(keyName + " (tps)", defaultValue.value()));
}

units::turns_per_second_squared_t Tune(std::string keyName, units::turns_per_second_squared_t defaultValue) {
  return units::turns_per_second_squared_t(Tune(keyName + " (tps_sq)", defaultValue.value()));
}

units::meter_t Tune(std::string keyName, units::meter_t defaultValue) {
  return units::meter_t(Tune(keyName + " (m)", defaultValue.value()));
}

units::meters_per_second_t Tune(std::string keyName, units::meters_per_second_t defaultValue) {
  return units::meters_per_second_t(Tune(keyName + " (mps)", defaultValue.value()));
}

units::meters_per_second_squared_t Tune(std::string keyName, units::meters_per_second_squared_t defaultValue) {
  return units::meters_per_second_squared_t(Tune(keyName + " (mps_sq)", defaultValue.value()));
}

units::volt_t Tune(std::string keyName, units::volt_t defaultValue) {
  return units::volt_t(Tune(keyName + " (V)", defaultValue.value()));
}

units::ampere_t Tune(std::string keyName, units::ampere_t defaultValue) {
  return units::ampere_t(Tune(keyName + " (A)", defaultValue.value()));
}

units::second_t Tune(std::string keyName, units::second_t defaultValue) {
  return units::second_t(Tune(keyName + " (s)", defaultValue.value()));
}

units::kilogram_t Tune(std::string keyName, units::kilogram_t defaultValue) {
  return units::kilogram_t(Tune(keyName + " (kg)", defaultValue.value()));
}

units::celsius_t Tune(std::string keyName, units::celsius_t defaultValue) {
  return units::celsius_t(Tune(keyName + " (C)", defaultValue.value()));
}

frc::Rotation2d Tune(std::string keyName, frc::Rotation2d defaultValue) {
  return frc::Rotation2d(Tune(keyName, defaultValue.Degrees()));
}

}  // namespace Logger
