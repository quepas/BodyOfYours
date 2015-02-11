#pragma once

#include "scanner_3d.h"

#include <reconstructmesdk/reme.h>

class RemeScanner3D : public Scanner3D
{
public:
  RemeScanner3D();
  ~RemeScanner3D();

  QStringList GetComputingDevices() override;
  bool InitComputingDevice(int device_id) override;
  bool GrabCameraFrame(FrameData* out_frame) override;
  bool GrabDepthFrame(FrameData* out_frame) override;

private:
  reme_context_t context_;
  reme_sensor_t sensor_;
  reme_viewer_t img_viewer_;
  reme_image_t image_aux_, image_depth_;
};
