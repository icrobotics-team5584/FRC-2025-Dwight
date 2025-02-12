// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/smartdashboard/SmartDashboard.h>
#include "subsystems/SubLaserCAN.h"

SubLaserCAN::SubLaserCAN() {
    laserCAN = new grpl::LaserCan(canid::LaserCAN);
};

std::string GetLCStatusString(uint8_t statusID) {
    switch (statusID) {
        case grpl::LASERCAN_STATUS_NOISE_ISSUE:
            return "Noise Issue";
        case grpl::LASERCAN_STATUS_OUT_OF_BOUNDS:
            return "Out of bounds";
        case grpl::LASERCAN_STATUS_VALID_MEASUREMENT:
            return "Valid measurement";
        case grpl::LASERCAN_STATUS_WEAK_SIGNAL:
            return "Weak signal";
        case grpl::LASERCAN_STATUS_WRAPAROUND:
            return "Wraparound";
        default:
            return "Unknown";
    }
}

// This method will be called once per scheduler run
void SubLaserCAN::Periodic() {
    std::optional<grpl::LaserCanMeasurement> measurement = laserCAN->get_measurement();
    if (measurement.has_value()) {
        frc::SmartDashboard::PutNumber("Last seen distance", measurement.value().distance_mm);
        frc::SmartDashboard::PutString("Measurement validity", "Valid");
        frc::SmartDashboard::PutString("LaserCAN Status", GetLCStatusString(measurement.value().status));
    } else {
        frc::SmartDashboard::PutString("Measurement validity", "No measurement");
    }
}
