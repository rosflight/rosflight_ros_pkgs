#include <pluginlib/class_list_macros.hpp>
#include <marble/MarbleWidget.h>
#include <marble/GeoDataPlacemark.h>
#include <marble/GeoDataDocument.h>
#include <marble/GeoDataGroundOverlay.h>
#include <marble/GeoDataTreeModel.h>
#include <marble/MarbleModel.h>
#include <QImage>

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

  // Initialize the marble widget
  widget_ = new QWidget();
  ui_.setupUi(widget_);
  // OpenTopoMap would be a better choice, but when I tested it I found downloading to be
  // slow and inconsistent compared to OpenStreetMap. Marble has additional .dgml files for
  // additional maps including OpenTopoMap found here: https://marble.kde.org/maps-4.5.php
  ui_.MarbleWidget->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
  ui_.MarbleWidget->setProjection(Marble::Mercator);
  ui_.MarbleWidget->centerOn(-111.901589, 40.363455, false); // Saratoga Springs RC park
  ui_.MarbleWidget->setDistance(0.05);
  context.addWidget(widget_);

  // Create image overlay object
  auto overlay = new Marble::GeoDataGroundOverlay;
  overlay->setIcon(QImage("/rosflight_ws/src/rosflight_ros_pkgs/rosflight_rqt_plugins/map_viewer/resources/plane.png"));

  // Set the geographical location of the image
  Marble::GeoDataLatLonBox latLonBox;
  latLonBox.setNorth(40.3636, Marble::GeoDataCoordinates::Degree);
  latLonBox.setSouth(40.3634, Marble::GeoDataCoordinates::Degree);
  latLonBox.setEast(-111.9014, Marble::GeoDataCoordinates::Degree);
  latLonBox.setWest(-111.9017, Marble::GeoDataCoordinates::Degree);
  latLonBox.setRotation(90, Marble::GeoDataCoordinates::Degree);
  overlay->setLatLonBox(latLonBox);

  // Add the overlay to a GeoDataDocument and then the marble widget
  auto document = new Marble::GeoDataDocument;
  document->append(overlay);
  ui_.MarbleWidget->model()->treeModel()->addDocument(document);
  ui_.MarbleWidget->show();
}

void MapView::shutdownPlugin() {}

} // namespace rosflight_rqt_plugins

PLUGINLIB_EXPORT_CLASS(map_view::MapView, rqt_gui_cpp::Plugin)
