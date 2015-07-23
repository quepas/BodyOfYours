#pragma once

#include <RecFusion.h>

struct SensorData
{
  RecFusion::ColorImage* color_image;
  RecFusion::DepthImage* depth_image;
  RecFusion::ColorImage* scene_image;
  RecFusion::ColorImage* calib_color_image;
  RecFusion::DepthImage* calib_depth_image;
  bool calib_image_valid;
  RecFusion::Mat3 K;
  RecFusion::Mat4 T;
};