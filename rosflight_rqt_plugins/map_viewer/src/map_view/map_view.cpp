#include <pluginlib/class_list_macros.hpp>

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
  widget_ = new QWidget();
  ui_.setupUi(widget_);

  if (context.serialNumber() > 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);
}

void MapView::shutdownPlugin() {}

} // namespace rosflight_rqt_plugins

PLUGINLIB_EXPORT_CLASS(map_view::MapView, rqt_gui_cpp::Plugin)
