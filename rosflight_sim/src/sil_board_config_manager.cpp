#include <cstring> //strcpy

#include "sil_board_config_manager.h"

namespace rosflight_sim
{
rosflight_firmware::hardware_config_t SILBoardConfigManager::get_max_config(rosflight_firmware::device_t device) const
{
  return 0;
}
rosflight_firmware::ConfigManager::ConfigResponse SILBoardConfigManager::check_config_change(rosflight_firmware::device_t device, rosflight_firmware::hardware_config_t, const rosflight_firmware::ConfigManager &cm) const
{
  rosflight_firmware::ConfigManager::ConfigResponse response;
  response.successful = false;
  response.reboot_required = false;
  strcpy(response.message, "Not supported in sim");
  return response;
}
void SILBoardConfigManager::get_device_name(rosflight_firmware::device_t device, char (&name)[rosflight_firmware::BoardConfigManager::DEVICE_NAME_LENGTH]) const
{
  strcpy(name,"Sim device");
}
void SILBoardConfigManager::get_config_name(rosflight_firmware::device_t device, rosflight_firmware::hardware_config_t config, char(&name)[rosflight_firmware::BoardConfigManager::CONFIG_NAME_LENGTH]) const
{
  strcpy(name, "Sim config");
}
} // namespace rosflight_sim