#include "frc/DataLogManager.h"
#include "wpi/DataLog.h"
#include <ctre/phoenix6/TalonFX.hpp>

namespace Logger {
    void logFalcon(ctre::phoenix6::hardware::TalonFX& talonFX, std::string name);
};