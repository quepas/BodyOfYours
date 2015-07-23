#include "SensorData.h"

using RecFusion::ColorImage;
using RecFusion::DepthImage;
using RecFusion::Sensor;

SensorData::SensorData(const Sensor& sensor)
{
  int w = sensor.width();
  int h = sensor.height();
  K = sensor.depthIntrinsics();
  color_image = new ColorImage(w, h);
  depth_image = new DepthImage(w, h);
  scene_image = new ColorImage(w, h);
  calib_color_image = new ColorImage(w, h);
  calib_depth_image = new DepthImage(w, h);
}

SensorData::~SensorData()
{
  delete color_image;
  delete depth_image;
  delete scene_image;
  delete calib_color_image;
  delete calib_depth_image;
}