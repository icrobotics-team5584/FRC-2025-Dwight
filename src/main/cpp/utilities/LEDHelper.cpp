#include "utilities/LEDHelper.h"
#include "utilities/RobotLogs.h"
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
        static double ledValue = 0.5;

        double change = (rand() / (double)RAND_MAX) * 0.1;
        // this generates a random number, scales it to [0,1] by dividing by the max random number,
        // then scales it further to [0,0.1]. i.e. the # of LEDs lit will move up or down up to 10%.

        double direction = (ledValue >= 0.98) ? -1 : //go DOWN if all LEDs are already lit
                              (ledValue <= 0.5) ? 1 : //go UP if lower threshold (50%) of LEDs are lit
                              (rand() % 2) * 2 - 1;  //otherwise randomly go UP or DOWN ([0 or 1] > [0 or 2] > [-1 or 1])

        ledValue += (change * direction);

        Logger::Log("LED value", ledValue);
        Logger::Log("change", change);
        Logger::Log("direction", direction);

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

void LEDHelper::Start(int length) {
    _led.SetLength(length);
    _ledBuffer.resize(length);
    _led.SetData(_ledBuffer);
    _led.Start();
}