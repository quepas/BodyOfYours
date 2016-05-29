#include "SensorData.h"
#include "RecFusionUtils.h"

using RecFusion::ColorImage;
using RecFusion::DepthImage;
using RecFusion::Sensor;

SensorData::SensorData(const Sensor& sensor)
{
  int w = sensor.depthWidth();
  int h = sensor.depthHeight();
  K = sensor.depthIntrinsics();
  T = identityMat4();
  color_image = new ColorImage(w, h);
  depth_image = new DepthImage(w, h);
  scene_image = new ColorImage(w, h);
}

SensorData::~SensorData()
{
  delete color_image;
  delete depth_image;
  delete scene_image;
}

bool SensorData::HasImages()
{
  return color_image && depth_image;
}

void SensorData::ResetT()
{
  T = identityMat4();
}

bool SensorData::IsCalibrated()
{
  return calib_image_valid;
}
