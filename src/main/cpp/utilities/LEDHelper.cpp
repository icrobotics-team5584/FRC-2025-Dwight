#include "utilities/LEDHelper.h"
#include <frc/util/Color.h>
#include <thread>


void LEDHelper::SetSolidColour(frc::Color color) {
    frc::LEDPattern ledpattern = frc::LEDPattern::Solid(color);
    ledpattern.ApplyTo(m_ledBuffer);
    m_led.SetData(m_ledBuffer);
}

void LEDHelper::SetScrollingRainbow() {
    frc::LEDPattern m_rainbow = frc::LEDPattern::Rainbow(255, 128);
    frc::LEDPattern m_scrollingRainbow = m_rainbow.ScrollAtAbsoluteSpeed(units::velocity::meters_per_second_t{1}, kLedSpacing); //might need to continuously apply in robot periodic??
    m_led.SetData(m_ledBuffer);
}

void LEDHelper::SetContinuousGradient(frc::Color Color1, frc::Color Color2) {
    std::array<frc::Color, 2> colors{Color1, Color2};
    frc::LEDPattern gradient = frc::LEDPattern::Gradient(frc::LEDPattern::GradientType::kContinuous, colors);

    // Apply the LED pattern to the data buffer
    gradient.ApplyTo(m_ledBuffer);
    m_led.SetData(m_ledBuffer);
}

void LEDHelper::SetFire() {
    double randomValue = 0.5 + static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / 0.5));

    std::array<frc::Color, 2> colors{frc::Color::kRed, frc::Color::kWhite};
    frc::LEDPattern base = frc::LEDPattern::Gradient(frc::LEDPattern::GradientType::kContinuous, colors);
    frc::LEDPattern mask = frc::LEDPattern::ProgressMaskLayer([&]() { return randomValue; });
    
    frc::LEDPattern heightDisplay = base.Mask(mask);

    // Apply the LED pattern to the data buffer
    heightDisplay.ApplyTo(m_ledBuffer);
    m_led.SetData(m_ledBuffer);
}

void LEDHelper::SetBreatheColour(frc::Color color) {

    frc::LEDPattern base = frc::LEDPattern::Solid(color);
    frc::LEDPattern pattern = base.Breathe(2_s);

    // Apply the LED pattern to the data buffer
    pattern.ApplyTo(m_ledBuffer);
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