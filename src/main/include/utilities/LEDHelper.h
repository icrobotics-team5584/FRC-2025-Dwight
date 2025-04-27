#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/AddressableLED.h>
#include <frc/LEDPattern.h>
#include <units/length.h>

class LEDHelper : public frc2::SubsystemBase {  // made LEDHelper into a subsystem
public:
  static LEDHelper& GetInstance() {static LEDHelper inst; return inst;}


  void Start();

  void SetSolidColour(frc::Color color);
  void SetScrollingRainbow();
  void SetContinuousGradient(frc::Color Color1, frc::Color Color2);
  void SetBreatheColour(frc::Color color);
  void SetFollowProgress(double Progress);
  void SetFire();
  void FlashColour(frc::Color color);

private:
  static constexpr int kLength = 60;
  units::meter_t kLedSpacing = (1/50)*1_m; // CHANGE LATER this is for 50 leds every meter


  // PWM port 9 as a dummy value
  // Must be a PWM header, not MXP or DIO
  frc::AddressableLED m_led{9};
  std::array<frc::AddressableLED::LEDData, kLength>
      m_ledBuffer;  


};

//   m_led.SetLength(kLength);
//   m_led.SetData(m_ledBuffer);
//   m_led.Start();