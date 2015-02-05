#pragma once

#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <QVTKWidget.h>

class ScansViewer
{
public:
  ScansViewer(QVTKWidget* qvtk_widget);
  ~ScansViewer();

  bool ShowPointCloud(pcl::PointCloud<pcl::PointXYZ>::ConstPtr point_cloud);
private:
  pcl::visualization::PCLVisualizer visualizer_;
  QVTKWidget* qvtk_widget_;
  bool point_cloud_loaded_;
};
