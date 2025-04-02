#pragma once

#include <frc/AddressableLED.h>

class LEDHelper {
public:
  void Start();

private:
  static constexpr int kLength = 60;

  // PWM port 9 as a dummy value
  // Must be a PWM header, not MXP or DIO
  frc::AddressableLED m_led{9};
  std::array<frc::AddressableLED::LEDData, kLength>
      m_ledBuffer;  


};

//   m_led.SetLength(kLength);
//   m_led.SetData(m_ledBuffer);
//   m_led.Start();