#ifndef MAP_VIEW_HPP
#define MAP_VIEW_HPP

#include <rqt_gui_cpp/plugin.h>
#include <QWidget>

#include <ui_map_view.h>

namespace map_view
{
class MapView : public rqt_gui_cpp::Plugin
{
  Q_OBJECT

public:
  MapView();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();

private:
  Ui::MapViewWidget ui_;
  QWidget* widget_;
};
}  // namespace rosflight_rqt_plugins

#endif  // MAP_VIEW_HPP
