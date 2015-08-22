#include "SensorData.h"
#include "RecFusionUtils.h"

using RecFusion::ColorImage;
using RecFusion::DepthImage;
using RecFusion::Sensor;

SensorData::SensorData(const Sensor& sensor)
{
  int w = sensor.width();
  int h = sensor.height();
  K = sensor.depthIntrinsics();
  T = IdentityMat4();
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

bool SensorData::HasRegularImages()
{
  return color_image && depth_image;
}

bool SensorData::HasCalibrationImages()
{
  return calib_color_image && calib_depth_image;
}

void SensorData::ResetT()
{
  T = IdentityMat4();
}

bool SensorData::IsCalibrated()
{
  return calib_image_valid;
}
