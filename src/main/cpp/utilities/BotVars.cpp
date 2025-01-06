#include "utilities/BotVars.h"
#include <iostream>

BotVars::Robot BotVars::DetermineRobot() {
  std::string filePath = "/sys/class/net/eth0/address";
  std::cout << "Running DetermineRobot()\n";
  frc::SmartDashboard::PutString("botVars/comp bot MAC address", COMP_BOT_MAC_ADDRESS);
  frc::SmartDashboard::PutString("botVars/prac bot MAC address", PRACTICE_BOT_MAC_ADDRESS);
  if (std::filesystem::exists(filePath)) {
    std::ifstream file(filePath);
    std::string macAddress;
    file >> macAddress;
    frc::SmartDashboard::PutString("botVars/MAC address", macAddress);
    if (macAddress == COMP_BOT_MAC_ADDRESS) {
      frc::SmartDashboard::PutString("botVars/active robot", "COMP");
      return Robot::COMP;
    } else if (macAddress == PRACTICE_BOT_MAC_ADDRESS) {
      frc::SmartDashboard::PutString("botVars/active robot", "PRACTICE");
      return Robot::PRACTICE;
    }
  }
  frc::SmartDashboard::PutString("botVars/active robot",
                                 "ERROR! Could not match MAC address. Defaulting to COMP Bot.");
  return Robot::COMP;
}


BotVars::Robot BotVars::GetRobot(){
  static Robot activeRobot = DetermineRobot();
  return activeRobot;
}
