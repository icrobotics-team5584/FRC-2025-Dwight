// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SubClimber.h"

#include <units/voltage.h>
#include "frc/smartdashboard/SmartDashboard.h"


SubClimber::SubClimber() {

    frc::SmartDashboard::PutData("Climber/Motor", &_climberMotor);
    

    _climberMotorConfig.encoder.PositionConversionFactor(1/GEAR_RATIO); // change to just GEAR_RATIO instead of 1/GEAR_RATIO for simulation
    _climberMotorConfig.encoder.VelocityConversionFactor(GEAR_RATIO/60.0);
    _climberMotorConfig.closedLoop.Pid(P, I, D, rev::spark::ClosedLoopSlot::kSlot0);
   // _climberMotorConfig.SecondaryCurrentLimit(0, 100);
    // _climberMotorConfig.closedLoop.PositionWrappingEnabled(true)
    //     .PositionWrappingMinInput(-10000000)
    //     .PositionWrappingMaxInput(10000000);
    _climberMotorConfig.Inverted(true)
        .SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
    auto err = _climberMotor.AdjustConfig(_climberMotorConfig);
    frc::SmartDashboard::PutNumber("Climber/config set err", (int)err);

}

// This method will be called once per scheduler run
void SubClimber::Periodic() {
    frc::SmartDashboard::PutNumber("Climber/MotorTarget", _climberMotor.GetPositionTarget().value());
    frc::SmartDashboard::PutData("Climber/armMechDisplay", &_singleJointedArmMech);
    frc::SmartDashboard::PutNumber("Climber/MotorVoltage", _climberMotor.GetMotorVoltage().value());
    frc::SmartDashboard::PutNumber("Climber/Current", _climberMotor.GetOutputCurrent());
    frc::SmartDashboard::PutNumber("Climber/PositionMotor", (_climberMotor.GetPosition().value()));
    frc::SmartDashboard::PutNumber("Climber/PositionArm", ((_climberMotor.GetPosition().value())/GEAR_RATIO));
    frc::SmartDashboard::PutNumber("Climber/M1Current", GetM1Current().value());
    frc::SmartDashboard::PutNumber("Climber/M1Reset", ResetM1);
    frc::SmartDashboard::PutNumber("Climber/M1Reseting", Reseting);

    if (ResetM1 == false && Reseting == false) {
        _climberMotor.Set(0);
    }

}

void SubClimber::SetBrakeMode(bool mode){
    if (mode == true) {
        _climberMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kBrake);
        _climberMotor.AdjustConfigNoPersist(_climberMotorConfig);
    } else if (mode == false) {
        _climberMotorConfig.SetIdleMode(rev::spark::SparkBaseConfig::IdleMode::kCoast);
        _climberMotor.AdjustConfigNoPersist(_climberMotorConfig);
    }

}

bool SubClimber::IsAtTarget() {
    units::turn_t currentRotation = _climberMotor.GetPosition();
    units::turn_t targetRotation = _climberMotor.GetPositionTarget();
    units::turn_t tolerance = 0.05_deg;
    if (currentRotation > targetRotation - tolerance && currentRotation < targetRotation + tolerance) {
        return true;
    }
    else {
        return false;
    }
}

//Auto climber reset by bringing Climber to zero position then reset
frc2::CommandPtr SubClimber::ClimberAutoReset() {
    return frc2::cmd::RunOnce([this] {
        Reseting = true; 
        rev::spark::SparkBaseConfig config;
        //config.SecondaryCurrentLimit(80, 0);
        config.SmartCurrentLimit(60);
        _climberMotor.AdjustConfigNoPersist(config); })

        .AndThen(ManualClimberMovementDOWNSLOW())
        .AndThen(ClimberResetCheck())
        .AndThen([this] {_climberMotor.SetPosition(0_deg);})
        .FinallyDo([this] {
            _climberMotor.StopMotor();
            if (ResetM1 == false) {
                rev::spark::SparkBaseConfig config;
                config.SmartCurrentLimit(0);
            //    config.SecondaryCurrentLimit(0, 100);
                _climberMotor.AdjustConfigNoPersist(config); 
            }
            Reseting = false;
        });
}

frc2::CommandPtr SubClimber::ClimberResetCheck() {
    return frc2::cmd::RunOnce ([this] {ResetM1 = false;})
    .AndThen(
    frc2::cmd::Run([this] {
        
        if (GetM1Current() > zeroingCurrentLimit) {
            ResetM1 = true;
        }
        else {
            Reseting = false;
        }
    }).Until([this] { return ResetM1; }));
}

frc2::CommandPtr SubClimber::ManualClimberMovementUP() {
  return frc2::cmd::StartEnd(
      [this] { _climberMotor.SetVoltage(4_V); },
      [this] {
        auto targRot = _climberMotor.GetPosition();
        _climberMotor.SetMaxMotionTarget(targRot);
      });
}

frc2::CommandPtr SubClimber::ManualClimberMovementDOWN() {
  return frc2::cmd::StartEnd(
      [this] { _climberMotor.SetVoltage(-4_V); },
      [this] {
        auto targRot = _climberMotor.GetPosition();
        _climberMotor.SetMaxMotionTarget(targRot);
      });
}

frc2::CommandPtr SubClimber::ManualClimberMovementDOWNSLOW() {
  return frc2::cmd::RunOnce(
      [this] { _climberMotor.SetVoltage(-1_V); });
}

    

units::ampere_t SubClimber::GetM1Current() {
    return _climberMotor.GetOutputCurrent()*1_A;
}

frc2::CommandPtr SubClimber::StowClimber() {
    return frc2::cmd::RunOnce([this] {_climberMotor.SetPositionTarget(STOW_TURNS);});
}

frc2::CommandPtr SubClimber::ReadyClimber() {
    return frc2::cmd::RunOnce([this] {_climberMotor.SetPositionTarget(PREPARE_TURNS);});
}

frc2::CommandPtr SubClimber::Climb() {
    return frc2::cmd::RunOnce([this] {_climberMotor.SetPositionTarget(CLIMB_TURNS);});
}

frc2::CommandPtr SubClimber::set12V() {
    return frc2::cmd::RunOnce([this] {_climberMotor.SetVoltage(12_V);});
}

void SubClimber::SimulationPeriodic() {
    units::angle::degree_t pivotAngle = _climberMotor.GetPosition();
    _arm1Ligament->SetAngle(pivotAngle);
    _climberSim.SetInputVoltage(_climberMotor.CalcSimVoltage());
    _climberSim.Update(20_ms);

    auto armAngle = _climberSim.GetAngle();
    auto armVel = _climberSim.GetVelocity();
    _climberMotor.IterateSim(armVel);

    frc::SmartDashboard::PutNumber("Climber/armAngle", armAngle.value());
    frc::SmartDashboard::PutNumber("Climber/armVelocity", armVel.value());
    frc::SmartDashboard::PutNumber("Climber/SimVoltage", _climberMotor.CalcSimVoltage().value());
}   