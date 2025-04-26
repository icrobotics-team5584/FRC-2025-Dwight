#include "utilities/LEDHelper.h"
#include <frc/util/Color.h>
#include <thread>


void LEDHelper::SetSolidColour(frc::Color color) {
    for (int i = 0; i < kLength; i++) {
        m_ledBuffer[i].SetRGB(color.red, color.green, color.blue);
    }
    m_led.SetData(m_ledBuffer);
}

void LEDHelper::SetRainbow() {
    static int rainbowFirstPixelHue = 0;
    for (int i = 0; i < kLength; i++) {
        const int hue = (rainbowFirstPixelHue + (i * 180 / kLength)) % 180;
        m_ledBuffer[i].SetHSV(hue, 255, 128);
    }
    rainbowFirstPixelHue += 3;
    rainbowFirstPixelHue %= 180;
    m_led.SetData(m_ledBuffer);
}

void LEDHelper::FlashColour(frc::Color color) {
    for (int i = 0; i < kLength; i++) {
        m_ledBuffer[i].SetRGB(color.red, color.green, color.blue);
    }
    m_led.SetData(m_ledBuffer);
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    for (int i = 0; i < kLength; i++) {
        m_ledBuffer[i].SetRGB(0, 0, 0); // Turn off LEDs
    }
    m_led.SetData(m_ledBuffer);
}

void LEDHelper::Start() {
    m_led.SetLength(kLength);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
}