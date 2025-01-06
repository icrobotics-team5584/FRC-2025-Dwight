#include <frc/smartdashboard/SmartDashboard.h>
#include "utilities/RobotLogs.h"

namespace Logger {
    void logFalcon(ctre::phoenix6::hardware::TalonFX& talonFX, std::string name) {
        std::string logPath = "FalconLogs/" + name;
        frc::SmartDashboard::PutNumber(logPath + "/Voltage", talonFX.GetMotorVoltage().GetValue().value());
        frc::SmartDashboard::PutNumber(logPath + "/StatorCurrent", talonFX.GetStatorCurrent().GetValue().value());
        frc::SmartDashboard::PutNumber(logPath + "/SupplyCurrent", talonFX.GetSupplyCurrent().GetValue().value());
        frc::SmartDashboard::PutNumber(logPath + "/Position", talonFX.GetPosition().GetValue().value());
        frc::SmartDashboard::PutNumber(logPath + "/Temperature", talonFX.GetDeviceTemp().GetValue().value());
        frc::SmartDashboard::PutNumber(logPath + "/TrueVelocity", talonFX.GetVelocity().GetValue().value());
        frc::SmartDashboard::PutNumber(logPath + "/TargetVelocity", talonFX.GetClosedLoopReference().GetValue());
    }
}
