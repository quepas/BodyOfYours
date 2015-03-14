#pragma once

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/PolygonMesh.h>
#include <QVTKWidget.h>

class ScansViewer : public QVTKWidget
{
public:
  ScansViewer(QWidget* parent = nullptr);
  ~ScansViewer();

  bool ShowPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr point_cloud);
  void ShowMesh(pcl::PolygonMesh mesh);
private:
  pcl::visualization::PCLVisualizer visualizer_;
  bool point_cloud_loaded_, mesh_loaded_;
};
