#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <units/length.h>
#include <frc2/command/Commands.h>
#include "Constants.h"
#include <functional>

class LEDHelper : public frc2::SubsystemBase { 
public:
  static LEDHelper& GetInstance() {static LEDHelper inst; return inst;}
  void Start(int length);

  frc2::CommandPtr SetSolidColour(frc::Color color);
  frc2::CommandPtr SetScrollingRainbow();
  frc2::CommandPtr SetContinuousGradient(frc::Color color1, frc::Color color2);
  frc2::CommandPtr SetBreatheColour(frc::Color color);
  frc2::CommandPtr SetFollowProgress(std::function <double()> progress, frc::Color color);
  frc2::CommandPtr SetFire(int cooldownIntensity=25, int lastCellMinimumHeat=60, int chanceOfSpark=8, units::hertz_t animationFrequency=30_Hz);
  frc::Color HeatColor(uint8_t temperature);
  frc2::CommandPtr FlashColour(frc::Color color);

private:
  frc::AddressableLED _led{pwm::LED};
  std::vector<frc::AddressableLED::LEDData> _ledBuffer;
  int _length = 0; //gets changed in Start()
  std::vector<uint8_t> _heat;
};