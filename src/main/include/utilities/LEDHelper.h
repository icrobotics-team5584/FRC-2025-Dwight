#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <units/length.h>
#include <frc2/command/Commands.h>
#include "Constants.h"

class LEDHelper : public frc2::SubsystemBase { 
public:
  static LEDHelper& GetInstance() {static LEDHelper inst; return inst;}
  void Start(int length);

  frc2::CommandPtr SetSolidColour(frc::Color color);
  frc2::CommandPtr SetScrollingRainbow();
  frc2::CommandPtr SetContinuousGradient(frc::Color Color1, frc::Color Color2);
  frc2::CommandPtr SetBreatheColour(frc::Color color);
  frc2::CommandPtr SetFollowProgress(double Progress);
  frc2::CommandPtr SetFire();
  frc2::CommandPtr FlashColour(frc::Color color);

private:
  frc::AddressableLED _led{pwm::LED};
  std::vector<frc::AddressableLED::LEDData> _ledBuffer;  

};