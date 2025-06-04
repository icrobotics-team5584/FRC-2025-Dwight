#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <units/length.h>
#include <frc2/command/Commands.h>

class LEDHelper : public frc2::SubsystemBase { 
public:
  static LEDHelper& GetInstance() {static LEDHelper inst; return inst;}


  void Start();

  frc2::CommandPtr SetSolidColour(frc::Color color);
  frc2::CommandPtr SetScrollingRainbow();
  frc2::CommandPtr SetContinuousGradient(frc::Color Color1, frc::Color Color2);
  frc2::CommandPtr SetBreatheColour(frc::Color color);
  frc2::CommandPtr SetFollowProgress(double Progress);
  frc2::CommandPtr SetFire();
  frc2::CommandPtr FlashColour(frc::Color color);

private:
  static constexpr int kLength = 17;
  units::meter_t kLedSpacing = (1/50)*1_m; // CHANGE LATER this is for 50 leds every meter


  // PWM port 9 as a dummy value
  // Must be a PWM header, not MXP or DIO
  frc::AddressableLED _led{9};
  std::array<frc::AddressableLED::LEDData, kLength>
      _ledBuffer;  


};

//   m_led.SetLength(kLength);
//   m_led.SetData(m_ledBuffer);
//   m_led.Start();