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
  bool GrabVolumeFrame(FrameData* out_frame) override;
  void RestartReconstruction() override;
  void ReconstructSurfaceToFile(QString file_name, int max_faces /* = 20000 */) override;

private:
  reme_context_t context_;
  reme_sensor_t sensor_;
  reme_image_t image_;
  reme_volume_t volume_;

  bool GrabFrame(FrameData* out_frame, reme_sensor_image_t frame_type);
};
