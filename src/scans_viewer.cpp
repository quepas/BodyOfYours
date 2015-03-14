#include "scans_viewer.h"

#include <vtkRenderWindow.h>

using pcl::PointCloud;
using pcl::PointXYZ;

ScansViewer::ScansViewer(QWidget* parent /*= nullptr*/)
  : QVTKWidget(parent),
    visualizer_("scans_viewer", false),
    point_cloud_loaded_(false),
    mesh_loaded_(false)
{
  vtkSmartPointer<vtkRenderWindow> render_window = visualizer_.getRenderWindow();
  SetRenderWindow(render_window);
  visualizer_.setupInteractor(GetInteractor(), GetRenderWindow());
  show();
}

ScansViewer::~ScansViewer()
{
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

void ScansViewer::ShowMesh(pcl::PolygonMesh mesh)
{
  if (mesh_loaded_) {
    visualizer_.updatePolygonMesh(mesh);
  } else {
    visualizer_.addPolygonMesh(mesh);
    mesh_loaded_ = true;
  }
}
