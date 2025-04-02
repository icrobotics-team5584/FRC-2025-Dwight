#include "utilities/LEDHelper.h"

void LEDHelper::Start() {
    m_led.SetLength(kLength);
    m_led.SetData(m_ledBuffer);
    m_led.Start();
}