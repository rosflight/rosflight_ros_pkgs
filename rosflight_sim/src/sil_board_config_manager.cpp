#include <cstring> //strcpy
#include <string>

#include "sil_board_config_manager.h"

using namespace rosflight_firmware;

namespace rosflight_sim
{
rosflight_firmware::hardware_config_t SILBoardConfigManager::get_max_config(rosflight_firmware::device_t device) const
{
  return 0;
}
rosflight_firmware::ConfigManager::ConfigResponse SILBoardConfigManager::check_config_change(rosflight_firmware::device_t device, rosflight_firmware::hardware_config_t config, const rosflight_firmware::ConfigManager &cm) const
{
  rosflight_firmware::ConfigManager::ConfigResponse response;
  if(config == 0)
  {
    response.successful = true;
    response.reboot_required = false;
    strcpy(response.message, "No change");
  }
  else
  {
    response.successful = false;
    response.reboot_required = false;
    strcpy(response.message, "Invalid configuration");
  }
  return response;
}
void SILBoardConfigManager::get_device_name(rosflight_firmware::device_t device, char (&name)[rosflight_firmware::BoardConfigManager::DEVICE_NAME_LENGTH]) const
{
  std::string device_name;
  switch(device)
  {
  case Configuration::SERIAL:
    device_name = "Serial";
    break;
  case Configuration::RC:
    device_name = "RC";
    break;
  case Configuration::AIRSPEED:
    device_name = "Airspeed";
    break;
  case Configuration::GNSS:
    device_name = "GNSS";
    break;
  case Configuration::SONAR:
    device_name = "Sonar";
    break;
  case Configuration::BATTERY_MONITOR:
    device_name = "Battery Monitor";
    break;
  case Configuration::BAROMETER:
    device_name = "Baro";
    break;
  case Configuration::MAGNETOMETER:
    device_name = "Mag";
    break;
  default:
    device_name = "Invalid Device";
    break;
  }
  strcpy(name, device_name.c_str());
}
void SILBoardConfigManager::get_config_name(rosflight_firmware::device_t device, rosflight_firmware::hardware_config_t config, char(&name)[rosflight_firmware::BoardConfigManager::CONFIG_NAME_LENGTH]) const
{
  std::string config_name;
  if(config == 0)
    switch(device)
    {
    case Configuration::SERIAL:
      config_name = "UDP";
      break;
    case Configuration::RC:
      config_name = "Simulated";
      break;
    case Configuration::AIRSPEED:
      config_name = "Simulated";
      break;
    case Configuration::GNSS:
      config_name = "Simulated";
      break;
    case Configuration::SONAR:
      config_name = "Simulated";
      break;
    case Configuration::BATTERY_MONITOR:
      config_name = "Simulated";
      break;
    case Configuration::BAROMETER:
      config_name = "Simulated";
      break;
    case Configuration::MAGNETOMETER:
      config_name = "Simulated";
      break;
    default:
      config_name = "Invalid device";
      break;
    }
  else
    config_name = "Invalid config";
  strcpy(name,config_name.c_str());
}
} // namespace rosflight_sim