#include "map_view/map_view.hpp"

namespace map_view
{

MapView::MapView()
{
  setObjectName("MapView");
}

void MapView::initPlugin(qt_gui_cpp::PluginContext& context) {}
void MapView::shutdownPlugin() {}
void MapView::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const {}
void MapView::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) {}

} // namespace rosflight_rqt_plugins