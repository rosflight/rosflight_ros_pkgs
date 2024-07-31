#ifndef MAP_VIEW_HPP
#define MAP_VIEW_HPP

#include <rqt_gui_cpp/plugin.h>

namespace map_view
{
class MapView : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  MapView();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);
};
}  // namespace rosflight_rqt_plugins

#endif  // MAP_VIEW_HPP