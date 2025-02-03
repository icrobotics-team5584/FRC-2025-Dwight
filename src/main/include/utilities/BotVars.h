#pragma once

#include <filesystem>
#include <fstream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <string>

namespace BotVars {

enum Robot { COMP, PRACTICE };
const inline std::string COMP_BOT_MAC_ADDRESS = "00:80:2f:34:07:fe";
const inline std::string PRACTICE_BOT_MAC_ADDRESS = "00:80:2f:33:d2:cb";

Robot DetermineRobot();
Robot GetRobot();


template <typename T>
T Choose(T compBotValue, T practiceBotValue) {
  return GetRobot() == COMP ? compBotValue : practiceBotValue;
}

}  // namespace BotVars