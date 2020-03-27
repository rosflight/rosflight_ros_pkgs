#include "board_config_manager.h"

namespace rosflight_sim
{
class SILBoardConfigManager: public rosflight_firmware::BoardConfigManager
{
  rosflight_firmware::hardware_config_t get_max_config(rosflight_firmware::device_t device) const override;
  rosflight_firmware::ConfigManager::ConfigResponse check_config_change(rosflight_firmware::device_t device, rosflight_firmware::hardware_config_t, const rosflight_firmware::ConfigManager &cm) const override;
  void get_device_name(rosflight_firmware::device_t device, char (&name)[rosflight_firmware::BoardConfigManager::DEVICE_NAME_LENGTH]) const override;
  void get_config_name(rosflight_firmware::device_t device, rosflight_firmware::hardware_config_t config, char(&name)[rosflight_firmware::BoardConfigManager::CONFIG_NAME_LENGTH]) const override;
};
} // namespace rosflight_sim