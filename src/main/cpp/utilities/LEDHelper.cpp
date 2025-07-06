#include "utilities/LEDHelper.h"
#include "utilities/RobotLogs.h"
#include <frc/util/Color.h>
#include <thread>
#include <functional>

frc2::CommandPtr LEDHelper::SetSolidColour(frc::Color color) {
  return RunOnce([this, color] {
           frc::LEDPattern ledpattern = frc::LEDPattern::Solid(color);
           ledpattern.ApplyTo(_ledBuffer);
           _led.SetData(_ledBuffer);
         })
      .AndThen(frc2::cmd::Idle());
}

frc2::CommandPtr LEDHelper::SetScrollingRainbow() {
  return Run([this] {
    frc::LEDPattern _rainbow = frc::LEDPattern::Rainbow(255, 128);
    frc::LEDPattern _scrollingRainbow = _rainbow.ScrollAtRelativeSpeed(1_Hz);
    _scrollingRainbow.ApplyTo(_ledBuffer);
    _led.SetData(_ledBuffer);
  });
}

frc2::CommandPtr LEDHelper::SetContinuousGradient(frc::Color color1, frc::Color color2) {
  return RunOnce([this, color1, color2] {
           std::array<frc::Color, 2> colors{color1, color2};
           frc::LEDPattern gradient =
               frc::LEDPattern::Gradient(frc::LEDPattern::GradientType::kContinuous, colors);

           // Apply the LED pattern to the data buffer
           gradient.ApplyTo(_ledBuffer);
           _led.SetData(_ledBuffer);
         })
      .AndThen(frc2::cmd::Idle());
}

frc2::CommandPtr LEDHelper::SetFire(int cooldownIntensity, int lastCellMinimumHeat,
                                    int chanceOfSpark, units::hertz_t animationFrequency) {
  return RunOnce([this, cooldownIntensity, lastCellMinimumHeat, chanceOfSpark] {
           // Cool down every cell a little
           for (int i = 0; i < _length; i++) {
             int cooldown = rand() % cooldownIntensity;  // higher = faster cooldown, lower = more
                                                         // constant. default is 25.
             _heat[i] = std::max(0, _heat[i] - cooldown);
           }

           // Heat diffusion upward (based on the two lower pixels)
           if (_length >= 3) {
             for (int i = _length - 1; i >= 2; i--) {
               _heat[i] = (_heat[i - 1] + _heat[i - 2] + _heat[i - 2]) / 3;
             }
           }

           // Handle bottom 2 pixels
           if (_length >= 2) {
             _heat[1] = (_heat[0] + _heat[0]) / 2;
           }  // second-last cell: manual heat diffusion based on last cell
           if (_length >= 1) {
             _heat[0] = std::max<uint8_t>(_heat[0] * 0.85, lastCellMinimumHeat);
             // last cell: heat decreases until minimum heat value. default is 60.
           }

           // Randomly ignite new heat near the bottom
           if (rand() % chanceOfSpark == 0) {  // chance of a spark starting. default is 1/8
             int y = rand() % (_length / 5);
             _heat[y] =
                 std::min(255, _heat[y] + rand() % 80 + 80);  // new spark intensity is between 80 and 140 + current cell heat. this can be changed as necessary
           }

           // Map heat to color
           for (int i = 0; i < _length; i++) {
             frc::Color color = HeatColor(_heat[i]);
             _ledBuffer[i].SetRGB(color.red * 255, color.green * 255, color.blue * 255);
           }

           _led.SetData(_ledBuffer);
         })
      .AndThen(frc2::cmd::Wait(((1.0 / animationFrequency.value()) * 1000_ms) - 20_ms)) // converts Hz to milliseconds, then compensates for the 20ms scheduler period
      .Repeatedly();
}

frc::Color LEDHelper::HeatColor(uint8_t heat) {
  if (heat < 85) {
    // Red (0–84): ramp up from black to red
    uint8_t red = heat * 3;
    return frc::Color{red / 255.0, 0.0, 0.0};
  } else if (heat < 170) {
    // Orange to yellow (85–169): red stays full, green ramps up
    uint8_t green = (heat - 85) * 3;
    return frc::Color{1.0, green / 255.0, 0.0};
  } else {
    // Yellow to white (170–255): red & green full, blue ramps up
    uint8_t blue = (heat - 170) * 3;
    return frc::Color{1.0, 1.0, blue / 255.0};
  }
}

frc2::CommandPtr LEDHelper::SetFollowProgress(std::function<double()> progress, frc::Color color) {
  return Run([this, progress, color] {
  double progressValue = progress();
  Logger::Log("LEDHelper/SetFollowProgress/Progress", progressValue);
    frc::LEDPattern base =
        frc::LEDPattern::Solid(color);
    // frc::LEDPattern mask = frc::LEDPattern::ProgressMaskLayer([&]() { return progress; });
    frc::LEDPattern mask = frc::LEDPattern::ProgressMaskLayer(progress);

    frc::LEDPattern heightDisplay = base.Mask(mask);

    // Apply the LED pattern to the data buffer
    heightDisplay.ApplyTo(_ledBuffer);
    _led.SetData(_ledBuffer);
  });
  // this can only be used for 0-1 progress
}

frc2::CommandPtr LEDHelper::SetBreatheColour(frc::Color color) {
  return Run([this, color] {
    frc::LEDPattern base = frc::LEDPattern::Solid(color);
    frc::LEDPattern pattern = base.Breathe(2_s);

    // Apply the LED pattern to the data buffer
    pattern.ApplyTo(_ledBuffer);
    _led.SetData(_ledBuffer);
  });
}

frc2::CommandPtr LEDHelper::FlashColour(frc::Color color) {
  return RunOnce([this, color] {
           frc::LEDPattern ledpattern = frc::LEDPattern::Solid(color);
           ledpattern.ApplyTo(_ledBuffer);
           _led.SetData(_ledBuffer);
         })
      .AndThen(frc2::cmd::Wait(200_ms));
}

void LEDHelper::Start(int length) {
  _led.SetLength(length);
  _ledBuffer.resize(length);
  _heat.resize(length, 0);
  _length = length;
  _led.SetData(_ledBuffer);
  _led.Start();
}