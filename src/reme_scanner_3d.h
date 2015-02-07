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
  void GrabCamera() override;
  void GrabDepth() override;

private:
  reme_context_t context_;
  reme_sensor_t sensor_;
  reme_viewer_t img_viewer_;
  reme_image_t image_aux_;
};
