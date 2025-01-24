#pragma once

namespace canid {
    constexpr int DriveBaseFrontRightDrive = 1; 
    constexpr int DriveBaseFrontRightTurn = 2;
    constexpr int DriveBaseFrontRightEncoder = 3; 

    constexpr int DriveBaseFrontLeftDrive = 4;
    constexpr int DriveBaseFrontLeftTurn = 5;
    constexpr int DriveBaseFrontLeftEncoder = 6;
        
    constexpr int DriveBaseBackRightDrive = 7; 
    constexpr int DriveBaseBackRightTurn = 8;
    constexpr int DriveBaseBackRightEncoder = 9; 

    constexpr int DriveBaseBackLeftDrive = 10; 
    constexpr int DriveBaseBackLeftTurn = 11;
    constexpr int DriveBaseBackLeftEncoder = 12; 

    constexpr int IntakeSpinningMotor = 15; 
    constexpr int EndEffectorMotor = 17; 
    constexpr int IntakePivotMotor = 18;


}

namespace dio {
    constexpr int EndEffectorLineBreakHigher = 1;  
    constexpr int EndEffectorLineBreakLower = 2;

    constexpr int toggleBrakeCoast = 3;
}