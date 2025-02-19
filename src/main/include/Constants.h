#pragma once

namespace canid {
    constexpr int DRIVEBASE_FRONT_RIGHT_DRIVE = 1; 
    constexpr int DRIVEBASE_FRONT_RIGHT_TURN = 2;
    constexpr int DRIVEBASE_FRONT_RIGHT_ENCODER = 3; 

    constexpr int DRIVEBASE_FRONT_LEFT_DRIVE = 4;
    constexpr int DRIVEBASE_FRONT_LEFT_TURN = 5;
    constexpr int DRIVEBASE_FRONT_LEFT_ENCODER = 6;
        
    constexpr int DRIVEBASE_BACK_RIGHT_DRIVE = 7; 
    constexpr int DRIVEBASE_BACK_RIGHT_TURN = 8;
    constexpr int DRIVEBASE_BACK_RIGHT_ENCODER = 9; 

    constexpr int DRIVEBASE_BACK_LEFT_DRIVE = 10; 
    constexpr int DRIVEBASE_BACK_LEFT_TURN = 11;
    constexpr int DRIVEBASE_BACK_LEFT_ENCODER = 12; 

    constexpr int INTAKE_MOTOR = 15;
    constexpr int ENDEFFECTOR_MOTOR = 16; 
    constexpr int INTAKE_PIVOT_MOTOR = 17;

    constexpr int ELEVATOR_MOTOR_1 = 13;
    constexpr int ELEVATOR_MOTOR_2 = 14;

    constexpr int CLIMBER_MOTOR = 18;

    constexpr int PIGEON_2 = 19; 

    constexpr int FUNNEL_MOTOR = 20;
}

namespace dio {
    constexpr int ENDEFFECTOR_LINEBREAK_HIGHER = 9;  
    constexpr int ENDEFFECTOR_LINEBREAK_LOWER = 8;
    constexpr int BRAKE_COAST_BUTTON = 4;
}

namespace math {
    constexpr double pi = 3.14159265358979323846;
}
