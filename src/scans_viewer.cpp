#include "scans_viewer.h"

#include <vtkRenderWindow.h>

using pcl::PointCloud;
using pcl::PointXYZ;

ScansViewer::ScansViewer(QVTKWidget* qvtk_widget)
  : visualizer_("scans_viewer", false),
    qvtk_widget_(qvtk_widget),
    point_cloud_loaded_(false)
{
  vtkSmartPointer<vtkRenderWindow> render_window = visualizer_.getRenderWindow();
  qvtk_widget_->SetRenderWindow(render_window);
  visualizer_.setupInteractor(qvtk_widget_->GetInteractor(), qvtk_widget_->GetRenderWindow());
  qvtk_widget_->show();
}

ScansViewer::~ScansViewer()
{
  visualizer_.close();
  qvtk_widget_->close();
}

bool ScansViewer::ShowPointCloud(PointCloud<PointXYZ>::ConstPtr point_cloud)
{
  static const std::string point_cloud_id = "point_cloud";
  if (point_cloud_loaded_) {
    visualizer_.updatePointCloud(point_cloud, point_cloud_id);
  } else {
    visualizer_.addPointCloud(point_cloud, point_cloud_id);
    point_cloud_loaded_ = true;
  }
  return true;
}
