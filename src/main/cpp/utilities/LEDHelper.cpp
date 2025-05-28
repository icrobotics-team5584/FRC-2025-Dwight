#include "utilities/LEDHelper.h"
#include <frc/util/Color.h>
#include <thread>

frc2::CommandPtr LEDHelper::SetSolidColour(frc::Color color) {
    return RunOnce([this, color] {
        frc::LEDPattern ledpattern = frc::LEDPattern::Solid(color);
        ledpattern.ApplyTo(_ledBuffer);
        _led.SetData(_ledBuffer);
    }).AndThen(frc2::cmd::Idle());

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
        frc::LEDPattern gradient = frc::LEDPattern::Gradient(frc::LEDPattern::GradientType::kContinuous, colors);

        // Apply the LED pattern to the data buffer
        gradient.ApplyTo(_ledBuffer);
        _led.SetData(_ledBuffer);
    }).AndThen(frc2::cmd::Idle());
}

frc2::CommandPtr LEDHelper::SetFire() {
    return Run([this] {
        static bool goingUp = true;
        static double ledValue = 0.5;

        if(ledValue > 0.98) {
            goingUp = false;
        }

        if(ledValue < 0.5) {
            goingUp = true;
        }

        if(goingUp) {
            ledValue += 0.1;
        }
        if(!goingUp) {
            ledValue -= 0.1;
        }

        std::array<frc::Color, 2> colors{frc::Color::kDarkRed, frc::Color::kDarkOrange};
        frc::LEDPattern base = frc::LEDPattern::Gradient(frc::LEDPattern::GradientType::kDiscontinuous, colors);
        frc::LEDPattern mask = frc::LEDPattern::ProgressMaskLayer([&]() { return ledValue; });
        
        frc::LEDPattern heightDisplay = base.Mask(mask);

        // Apply the LED pattern to the data buffer
        heightDisplay.ApplyTo(_ledBuffer);
        _led.SetData(_ledBuffer);
    });
}

frc2::CommandPtr LEDHelper::SetFollowProgress(double progress) {
    return RunOnce([this, progress] {
        std::array<frc::Color, 2> colors{frc::Color::kWhite, frc::Color::kGreen};
        frc::LEDPattern base = frc::LEDPattern::Gradient(frc::LEDPattern::GradientType::kContinuous, colors);
        frc::LEDPattern mask = frc::LEDPattern::ProgressMaskLayer([&]() { return progress; });
        
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
    }).AndThen(frc2::cmd::Wait(200_ms));
}

void LEDHelper::Start() {
    _led.SetLength(kLength);
    _led.SetData(_ledBuffer);
    _led.Start();
}