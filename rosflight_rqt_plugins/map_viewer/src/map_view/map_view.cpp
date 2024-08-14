#include <pluginlib/class_list_macros.hpp>
#include <marble/MarbleWidget.h>

#include "map_view/map_view.hpp"

namespace map_view
{

MapView::MapView()
  : rqt_gui_cpp::Plugin()
{
  setObjectName("MapView");
}

void MapView::initPlugin(qt_gui_cpp::PluginContext& context)
{
  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }

  widget_ = new QWidget();
  ui_.setupUi(widget_);
  ui_.MarbleWidget->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
  ui_.MarbleWidget->setProjection(Marble::Mercator);
  ui_.MarbleWidget->centerOn(-111.901589, 40.363455, false);  // Saratoga Springs RC park
  ui_.MarbleWidget->setDistance(0.05);
  context.addWidget(widget_);
}

void MapView::shutdownPlugin() {}

} // namespace rosflight_rqt_plugins

PLUGINLIB_EXPORT_CLASS(map_view::MapView, rqt_gui_cpp::Plugin)
